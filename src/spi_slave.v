`default_nettype none

module spi_slave (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       sck,
    input  wire       mosi,
    input  wire       cs_n,
    input  wire [7:0] status,
    input  wire [7:0] out_byte,
    input  wire       psram_ready,
    output wire       miso,
    output wire       start,
    output wire       mode,
    output wire       push,
    output wire [7:0] push_data,
    output wire       end_job,
    output wire       pop
);
    reg sck_meta;
    reg sck_sync;
    reg sck_prev;
    reg mosi_meta;
    reg mosi_sync;
    reg cs_meta;
    reg cs_sync;
    reg cs_prev;
    reg [3:0] cnt;
    reg [7:0] sr;
    reg [4:0] cmd;
    reg st_in_ready;
    reg st_out_valid;

    wire sck_rise = sck_sync && !sck_prev;
    wire cs_fall = cs_prev && !cs_sync;
    wire cs_rise = !cs_prev && cs_sync;
    wire command_edge = !cs_sync && sck_rise && (cnt == 4'hf);

    assign miso = sr[7];
    assign mode = cmd[3];
    assign push_data = {sr[6:0], mosi_sync};
    assign start = command_edge && cmd[4] && psram_ready;
    assign push = command_edge && !cmd[4] && cmd[0] && st_in_ready;
    assign end_job = command_edge && !cmd[4] && cmd[2] && st_in_ready;
    assign pop = command_edge && !cmd[4] && cmd[1] && st_out_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sck_meta <= 1'b0;
            sck_sync <= 1'b0;
            sck_prev <= 1'b0;
            mosi_meta <= 1'b0;
            mosi_sync <= 1'b0;
            cs_meta <= 1'b1;
            cs_sync <= 1'b1;
            cs_prev <= 1'b1;
            cnt <= 4'd0;
            cmd <= 5'd0;
            st_in_ready <= 1'b0;
            st_out_valid <= 1'b0;
        end else begin
            sck_meta <= sck;
            sck_sync <= sck_meta;
            sck_prev <= sck_sync;
            mosi_meta <= mosi;
            mosi_sync <= mosi_meta;
            cs_meta <= cs_n;
            cs_sync <= cs_meta;
            cs_prev <= cs_sync;

            if (cs_fall) begin
                cnt <= 4'd0;
                sr <= status;
                st_in_ready <= status[0];
                st_out_valid <= status[1];
            end else if (cs_rise) begin
                cnt <= 4'd0;
            end else if (!cs_sync && sck_rise) begin
                if (cnt == 4'd7) begin
                    cmd <= {sr[6], sr[5], sr[1], sr[0], mosi_sync};
                    sr <= st_out_valid ? out_byte : 8'd0;
                end else begin
                    sr <= {sr[6:0], mosi_sync};
                end

                if (cnt == 4'hf)
                    cmd <= 5'd0;
                else
                    cnt <= cnt + 1'b1;
            end
        end
    end
endmodule

`default_nettype wire
