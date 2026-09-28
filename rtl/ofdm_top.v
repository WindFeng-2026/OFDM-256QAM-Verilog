/*
 * OFDM Top Module - 1024 FFT, 16 CP, 256QAM, AXI-Stream Interface
 * Target: ZYNQ7100 FPGA
 * Author: OFDM Design Team
 * Date: 2026-09
 * 
 * Configuration:
 *  - FFT Points: 1024
 *  - Cyclic Prefix Length: 16
 *  - Data Subcarriers: 52
 *  - Pilot Subcarriers: 4 (at indices: 85, 213, 299, 427, 513, 641, 727, 855)
 *  - Modulation: 256QAM (8 bits per subcarrier)
 *  - Total Frame Length: 1024 + 16 = 1040 samples
 *  - AXI-Stream Interface: 32-bit data width
 */

module ofdm_top #(
    parameter FFT_SIZE = 1024,
    parameter CP_LEN = 16,
    parameter DATA_SUBCARRIER = 52,
    parameter PILOT_SUBCARRIER = 8,
    parameter TOTAL_SUBCARRIER = 60,
    parameter AXI_DATA_WIDTH = 32
)(
    // System Clock & Reset
    input wire clk,                          // System clock
    input wire rst_n,                        // Active low reset
    
    // TX AXI-Stream Interface (Data Input)
    input wire [AXI_DATA_WIDTH-1:0] tx_axis_tdata,
    input wire tx_axis_tvalid,
    output wire tx_axis_tready,
    input wire tx_axis_tlast,
    
    // TX Output (IQ Samples)
    output wire [31:0] tx_i_out,             // I component (16-bit signed)
    output wire [31:0] tx_q_out,             // Q component (16-bit signed)
    output wire tx_out_valid,
    
    // RX AXI-Stream Interface (Data Output)
    output wire [AXI_DATA_WIDTH-1:0] rx_axis_tdata,
    output wire rx_axis_tvalid,
    input wire rx_axis_tready,
    output wire rx_axis_tlast,
    
    // RX Input (IQ Samples)
    input wire [31:0] rx_i_in,               // I component (16-bit signed)
    input wire [31:0] rx_q_in,               // Q component (16-bit signed)
    input wire rx_in_valid,
    
    // Control & Status
    input wire [1:0] mode,                   // 00: TX, 01: RX, 10: RX+CFO Est, 11: Reserved
    output wire [3:0] status,                // [3]:tx_frame_done, [2]:rx_frame_done, [1]:rx_cfo_valid, [0]:error
    output wire [15:0] cfo_estimate          // Carrier Frequency Offset estimation result
);

    // =====================================================================
    // Signal Declaration
    // =====================================================================
    wire tx_ready, rx_ready;
    wire [15:0] tx_data_in;
    wire tx_data_valid;
    wire tx_data_last;
    wire [31:0] tx_iq_sample;
    wire tx_sample_valid;
    
    wire [31:0] rx_iq_sample;
    wire rx_sample_valid;
    wire [15:0] rx_data_out;
    wire rx_data_valid;
    wire rx_data_last;
    wire rx_frame_done;
    
    wire cfo_valid;
    wire [15:0] cfo_est;
    
    // =====================================================================
    // TX Path
    // =====================================================================
    
    // TX Data Buffer (AXI-Stream to parallel data)
    tx_data_buffer #(
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH)
    ) tx_buf_inst (
        .clk(clk),
        .rst_n(rst_n),
        .axi_tdata(tx_axis_tdata),
        .axi_tvalid(tx_axis_tvalid),
        .axi_tready(tx_axis_tready),
        .axi_tlast(tx_axis_tlast),
        .mode(mode),
        .data_out(tx_data_in),
        .data_valid(tx_data_valid),
        .data_last(tx_data_last)
    );
    
    // 256QAM Mapper (bits to IQ symbols)
    qam256_mapper tx_mapper_inst (
        .clk(clk),
        .rst_n(rst_n),
        .data_in(tx_data_in),
        .data_valid(tx_data_valid),
        .i_out(tx_i_out[31:16]),
        .q_out(tx_q_out[31:16]),
        .valid_out(tx_out_valid)
    );
    
    assign tx_i_out[15:0] = 16'h0;
    assign tx_q_out[15:0] = 16'h0;
    
    // TX OFDM Modulator (IFFT + CP + Serialization)
    tx_ofdm_modulator #(
        .FFT_SIZE(FFT_SIZE),
        .CP_LEN(CP_LEN),
        .DATA_SUBCARRIER(DATA_SUBCARRIER),
        .PILOT_SUBCARRIER(PILOT_SUBCARRIER),
        .TOTAL_SUBCARRIER(TOTAL_SUBCARRIER)
    ) tx_mod_inst (
        .clk(clk),
        .rst_n(rst_n),
        .enable(mode[0] == 1'b0),             // TX mode enabled when mode[0] = 0
        .i_in(tx_i_out[31:16]),
        .q_in(tx_q_out[31:16]),
        .valid_in(tx_out_valid),
        .i_out(tx_i_out[31:16]),
        .q_out(tx_q_out[31:16]),
        .valid_out(tx_sample_valid),
        .status(status[3])
    );
    
    // =====================================================================
    // RX Path
    // =====================================================================
    
    // RX Input Buffer
    rx_input_buffer rx_ibuf_inst (
        .clk(clk),
        .rst_n(rst_n),
        .i_in(rx_i_in[31:16]),
        .q_in(rx_q_in[31:16]),
        .valid_in(rx_in_valid),
        .i_out(rx_i_in[31:16]),
        .q_out(rx_q_in[31:16]),
        .valid_out(rx_sample_valid)
    );
    
    // CFO (Carrier Frequency Offset) Estimator
    cfo_estimator #(
        .FFT_SIZE(FFT_SIZE),
        .CP_LEN(CP_LEN)
    ) cfo_est_inst (
        .clk(clk),
        .rst_n(rst_n),
        .enable(mode[1] == 1'b1),             // CFO estimation enabled when mode[1] = 1
        .i_in(rx_i_in[31:16]),
        .q_in(rx_q_in[31:16]),
        .valid_in(rx_sample_valid),
        .cfo_out(cfo_est),
        .cfo_valid(cfo_valid)
    );
    
    assign cfo_estimate = cfo_est;
    assign status[1] = cfo_valid;
    
    // RX OFDM Demodulator (CP Removal + FFT + Equalization)
    rx_ofdm_demodulator #(
        .FFT_SIZE(FFT_SIZE),
        .CP_LEN(CP_LEN),
        .DATA_SUBCARRIER(DATA_SUBCARRIER),
        .PILOT_SUBCARRIER(PILOT_SUBCARRIER),
        .TOTAL_SUBCARRIER(TOTAL_SUBCARRIER)
    ) rx_demod_inst (
        .clk(clk),
        .rst_n(rst_n),
        .enable(mode[0] == 1'b1),             // RX mode enabled when mode[0] = 1
        .i_in(rx_i_in[31:16]),
        .q_in(rx_q_in[31:16]),
        .valid_in(rx_sample_valid),
        .cfo_correction(cfo_est),
        .i_out(rx_i_in[31:16]),
        .q_out(rx_q_in[31:16]),
        .valid_out(rx_sample_valid),
        .frame_done(rx_frame_done)
    );
    
    // 256QAM Demapper (IQ symbols to bits)
    qam256_demapper rx_demapper_inst (
        .clk(clk),
        .rst_n(rst_n),
        .i_in(rx_i_in[31:16]),
        .q_in(rx_q_in[31:16]),
        .valid_in(rx_sample_valid),
        .data_out(rx_data_out),
        .valid_out(rx_data_valid)
    );
    
    // RX Data Serializer (Parallel data to AXI-Stream)
    rx_data_serializer #(
        .AXI_DATA_WIDTH(AXI_DATA_WIDTH)
    ) rx_ser_inst (
        .clk(clk),
        .rst_n(rst_n),
        .data_in(rx_data_out),
        .data_valid(rx_data_valid),
        .frame_done(rx_frame_done),
        .axi_tdata(rx_axis_tdata),
        .axi_tvalid(rx_axis_tvalid),
        .axi_tready(rx_axis_tready),
        .axi_tlast(rx_axis_tlast)
    );
    
    assign status[2] = rx_frame_done;
    assign status[3] = tx_sample_valid;
    assign status[0] = 1'b0;  // Error flag (unused in this version)

endmodule
