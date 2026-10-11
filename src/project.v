`default_nettype none

module tt_um_rhgndf_lz4 (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

    reg rd_late;
    wire miso;
    wire start;
    wire mode;
    wire push;
    wire [7:0] push_data;
    wire end_job;
    wire pop;
    wire [7:0] status;
    wire in_ready;
    wire out_valid;
    wire done;
    wire error;
    wire [7:0] out_byte;
    wire ps_start;
    wire ps_we;
    wire [17:0] ps_addr;
    wire ps_more;
    wire psram_ready;
    wire psram_busy;
    wire ps_bdone;
    wire [3:0] ps_wnib;
    wire ps_nstrobe;
    wire [3:0] sd_in;
    wire [3:0] sd_out;
    wire [3:0] sd_oe;
    wire psram_sck;
    wire psram_cs_n;
    wire _unused = &{
        ena,
        ui_in[7:4],
        uio_in[7],
        uio_in[6],
        uio_in[3],
        uio_in[0],
        1'b0
    };

    assign sd_in = {uio_in[5], uio_in[4], uio_in[2], uio_in[1]};
    assign uio_out = {
        1'b1,
        psram_cs_n,
        sd_out[3],
        sd_out[2],
        psram_sck,
        sd_out[1],
        sd_out[0],
        1'b1
    };
    assign uio_oe = {
        1'b1,
        1'b1,
        sd_oe[3],
        sd_oe[2],
        1'b1,
        sd_oe[1],
        sd_oe[0],
        1'b1
    };
    assign status = {4'b1010, error, done, out_valid, in_ready};
    assign uo_out = {3'b000, error, done, out_valid, in_ready, miso};

    always @(posedge clk) begin
        if (!rst_n)
            rd_late <= ui_in[3];
    end

    spi_slave spi (
        .clk(clk),
        .rst_n(rst_n),
        .sck(ui_in[0]),
        .mosi(ui_in[1]),
        .cs_n(ui_in[2]),
        .status(status),
        .out_byte(out_byte),
        .psram_ready(psram_ready),
        .miso(miso),
        .start(start),
        .mode(mode),
        .push(push),
        .push_data(push_data),
        .end_job(end_job),
        .pop(pop)
    );

    psram_qpi psram (
        .clk(clk),
        .rst_n(rst_n),
        .rd_late(rd_late),
        .start(ps_start),
        .we(ps_we),
        .addr(ps_addr),
        .more(ps_more),
        .wnib(ps_wnib),
        .psram_sck(psram_sck),
        .psram_cs_n(psram_cs_n),
        .sd_out(sd_out),
        .sd_oe(sd_oe),
        .ready(psram_ready),
        .busy(psram_busy),
        .byte_done(ps_bdone),
        .nib_strobe(ps_nstrobe)
    );

    lz4_engine engine (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .mode(mode),
        .push(push),
        .push_data(push_data),
        .end_job(end_job),
        .pop(pop),
        .psram_ready(psram_ready),
        .psram_busy(psram_busy),
        .ps_bdone(ps_bdone),
        .ps_nstrobe(ps_nstrobe),
        .ps_sd_in(sd_in),
        .ps_start(ps_start),
        .ps_we(ps_we),
        .ps_addr(ps_addr),
        .ps_wnib(ps_wnib),
        .ps_more(ps_more),
        .in_ready(in_ready),
        .out_valid(out_valid),
        .done(done),
        .error(error),
        .out_byte(out_byte)
    );

endmodule

`default_nettype wire
