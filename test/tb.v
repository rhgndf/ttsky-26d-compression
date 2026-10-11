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
    wire [3:0] psram_sd_raw;
    wire [3:0] psram_sd_delayed;
    wire [3:0] psram_sd_bus;

    assign host_sd = {uio_out[5], uio_out[4], uio_out[2], uio_out[1]};
    assign host_sd_oe = {uio_oe[5], uio_oe[4], uio_oe[2], uio_oe[1]};
    assign #(PSRAM_READ_DELAY_NS) psram_sd_delayed = psram_sd_raw;
    assign psram_sd_bus = {
        host_sd_oe[3] ? host_sd[3] : psram_sd_delayed[3],
        host_sd_oe[2] ? host_sd[2] : psram_sd_delayed[2],
        host_sd_oe[1] ? host_sd[1] : psram_sd_delayed[1],
        host_sd_oe[0] ? host_sd[0] : psram_sd_delayed[0]
    };

    assign uio_in[0] = 1'b0;
    assign uio_in[1] = psram_sd_bus[0];
    assign uio_in[2] = psram_sd_bus[1];
    assign uio_in[3] = 1'b0;
    assign uio_in[4] = psram_sd_bus[2];
    assign uio_in[5] = psram_sd_bus[3];
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

    spiram psram_model (
        .spi_clk(uio_out[3]),
        .spi_ncs(uio_out[6]),
        .spi_sin(psram_sd_bus),
        .spi_sout(psram_sd_raw),
        .spi_oe(|host_sd_oe)
    );

`ifndef __pnr__
    initial begin
        $dumpfile("tb.vcd");
        $dumpvars(0, tb);
    end
`endif

endmodule

`default_nettype wire
