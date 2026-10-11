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
    output reg        start,
    output reg        mode,
    output reg        push,
    output reg  [7:0] push_data,
    output reg        end_job,
    output reg        pop
);

    reg sck_meta;
    reg sck_sync;
    reg sck_prev;
    reg mosi_meta;
    reg mosi_sync;
    reg cs_meta;
    reg cs_sync;
    reg cs_prev;
    reg [4:0] bit_count;
    reg [7:0] command_shift;
    reg [6:0] data_shift;
    reg [7:0] tx_shift;
    reg       status_in_ready;
    reg       status_out_valid;

    wire sck_rise = sck_sync && !sck_prev;
    wire sck_fall = !sck_sync && sck_prev;
    wire cs_fall = cs_prev && !cs_sync;
    wire cs_rise = !cs_prev && cs_sync;
    wire [7:0] received_data = {data_shift, mosi_sync};

    assign miso = tx_shift[7];

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
            bit_count <= 5'd0;
            command_shift <= 8'd0;
            data_shift <= 7'd0;
            tx_shift <= 8'd0;
            status_in_ready <= 1'b0;
            status_out_valid <= 1'b0;
            start <= 1'b0;
            mode <= 1'b0;
            push <= 1'b0;
            push_data <= 8'd0;
            end_job <= 1'b0;
            pop <= 1'b0;
        end else begin
            sck_meta <= sck;
            sck_sync <= sck_meta;
            sck_prev <= sck_sync;
            mosi_meta <= mosi;
            mosi_sync <= mosi_meta;
            cs_meta <= cs_n;
            cs_sync <= cs_meta;
            cs_prev <= cs_sync;

            start <= 1'b0;
            push <= 1'b0;
            end_job <= 1'b0;
            pop <= 1'b0;

            if (cs_fall) begin
                bit_count <= 5'd0;
                command_shift <= 8'd0;
                data_shift <= 7'd0;
                tx_shift <= status;
                status_in_ready <= status[0];
                status_out_valid <= status[1];
            end else if (cs_rise) begin
                bit_count <= 5'd0;
                tx_shift <= 8'd0;
            end else if (!cs_sync) begin
                if (sck_rise) begin
                    if (bit_count < 5'd8)
                        command_shift <= {command_shift[6:0], mosi_sync};
                    else if (bit_count < 5'd16)
                        data_shift <= received_data[6:0];

                    if (bit_count == 5'd15) begin
                        if (command_shift[7]) begin
                            if (psram_ready) begin
                                start <= 1'b1;
                                mode <= command_shift[6];
                            end
                        end else begin
                            if (command_shift[0] && status_in_ready) begin
                                push <= 1'b1;
                                push_data <= received_data;
                            end
                            if (command_shift[2] && status_in_ready)
                                end_job <= 1'b1;
                            if (command_shift[1] && status_out_valid)
                                pop <= 1'b1;
                        end
                    end

                    if (bit_count < 5'd16)
                        bit_count <= bit_count + 5'd1;
                end

                if (sck_fall) begin
                    if (bit_count == 5'd8)
                        tx_shift <= out_byte;
                    else if ((bit_count > 5'd0) && (bit_count < 5'd8))
                        tx_shift <= {tx_shift[6:0], 1'b0};
                    else if ((bit_count > 5'd8) && (bit_count < 5'd16))
                        tx_shift <= {tx_shift[6:0], 1'b0};
                end
            end
        end
    end

endmodule

`default_nettype wire
