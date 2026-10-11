`timescale 1ns/1ps
`default_nettype none

module tb #(
    parameter integer PSRAM_READ_DELAY_NS = 18
);

    reg clk;
    reg rst_n;
    reg [7:0] ui_in;
    wire [7:0] uo_out;
    wire [7:0] uio_out;
    wire [7:0] uio_oe;
    wire [7:0] uio_in;
    wire [3:0] host_sd;
    wire [3:0] host_sd_oe;
    wire [3:0] psram_sd_out;
    wire [3:0] psram_sd_oe;
    wire [31:0] psram_error_count;

    assign host_sd = {uio_out[5], uio_out[4], uio_out[2], uio_out[1]};
    assign host_sd_oe = {uio_oe[5], uio_oe[4], uio_oe[2], uio_oe[1]};

    assign uio_in[0] = 1'b0;
    assign uio_in[1] = uio_oe[1] ? uio_out[1] :
                       psram_sd_oe[0] ? psram_sd_out[0] : 1'bz;
    assign uio_in[2] = uio_oe[2] ? uio_out[2] :
                       psram_sd_oe[1] ? psram_sd_out[1] : 1'bz;
    assign uio_in[3] = 1'b0;
    assign uio_in[4] = uio_oe[4] ? uio_out[4] :
                       psram_sd_oe[2] ? psram_sd_out[2] : 1'bz;
    assign uio_in[5] = uio_oe[5] ? uio_out[5] :
                       psram_sd_oe[3] ? psram_sd_out[3] : 1'bz;
    assign uio_in[6] = 1'b0;
    assign uio_in[7] = 1'b0;

`ifdef USE_POWER_PINS
    wire VPWR = 1'b1;
    wire VGND = 1'b0;
`endif

    tt_um_rhgndf_lz4 user_project (
        .ui_in(ui_in),
        .uo_out(uo_out),
        .uio_in(uio_in),
        .uio_out(uio_out),
        .uio_oe(uio_oe),
        .ena(1'b1),
        .clk(clk),
        .rst_n(rst_n)
`ifdef USE_POWER_PINS
        ,.VPWR(VPWR)
        ,.VGND(VGND)
`endif
    );

    psram_model #(
        .READ_DELAY_NS(PSRAM_READ_DELAY_NS)
    ) psram_model (
        .sck(uio_out[3]),
        .cs_n(uio_out[6]),
        .host_sd(host_sd),
        .host_oe(host_sd_oe),
        .dev_sd(psram_sd_out),
        .dev_oe(psram_sd_oe),
        .error_count(psram_error_count)
    );

`ifndef __pnr__
    initial begin
        $dumpfile("tb.vcd");
        $dumpvars(0, tb);
    end
`endif

endmodule

`default_nettype wire
