`timescale 1ns/1ps
`default_nettype none

module psram_model #(
    parameter integer READ_DELAY_NS = 18
) (
    input  wire       sck,
    input  wire       cs_n,
    input  wire [3:0] host_sd,
    input  wire [3:0] host_oe,
    output reg  [3:0] dev_sd,
    output reg  [3:0] dev_oe,
    output reg [31:0] error_count
);

    localparam [2:0] M_IDLE      = 3'd0;
    localparam [2:0] M_COMMAND   = 3'd1;
    localparam [2:0] M_ADDR_READ = 3'd2;
    localparam [2:0] M_ADDR_WR   = 3'd3;
    localparam [2:0] M_DUMMY     = 3'd4;
    localparam [2:0] M_READ      = 3'd5;
    localparam [2:0] M_WRITE     = 3'd6;

    reg [7:0] mem [0:262143];
    reg [2:0] state;
    reg       qpi_mode;
    reg [7:0] spi_shift;
    reg [2:0] spi_bit_count;
    reg [7:0] qpi_shift;
    reg [2:0] nibble_count;
    reg [23:0] address_shift;
    reg [23:0] current_addr;
    reg [2:0] dummy_count;
    reg       read_phase;
    reg       write_phase;
    reg [3:0] write_high;
    reg [31:0] read_delay_ns;
    integer init_addr;
    time cs_low_start;
    wire [23:0] shifted_address = {address_shift[19:0], host_sd};

    task report_error;
        input [255:0] reason;
        begin
            $display("PSRAM ERROR: %0s at %0t", reason, $time);
            error_count = error_count + 1'b1;
        end
    endtask

`ifndef __pnr__
    initial begin
        error_count = 32'd0;
        dev_sd = 4'd0;
        dev_oe = 4'd0;
        state = M_IDLE;
        qpi_mode = 1'b0;
        spi_shift = 8'd0;
        spi_bit_count = 3'd0;
        qpi_shift = 8'd0;
        nibble_count = 3'd0;
        address_shift = 24'd0;
        current_addr = 24'd0;
        dummy_count = 3'd0;
        read_phase = 1'b0;
        write_phase = 1'b0;
        write_high = 4'd0;
        read_delay_ns = READ_DELAY_NS;
        cs_low_start = 0;
        for (init_addr = 0; init_addr < 262144; init_addr = init_addr + 1)
            mem[init_addr] = ((init_addr * 73) ^ (init_addr >> 7) ^
                              (init_addr >> 11) ^ ((init_addr >> 3) * 29) ^
                              32'h0000005a) & 8'hff;
    end
`endif

    always @(negedge cs_n) begin
        cs_low_start = $time;
        state = M_COMMAND;
        spi_shift = 8'd0;
        spi_bit_count = 3'd0;
        qpi_shift = 8'd0;
        nibble_count = 3'd0;
        address_shift = 24'd0;
        dummy_count = 3'd0;
        read_phase = 1'b0;
        write_phase = 1'b0;
        dev_oe = 4'd0;
    end

    always @(posedge cs_n) begin
        dev_oe = 4'd0;
        state = M_IDLE;
        if (($time - cs_low_start) > 8000)
            report_error("CS low longer than 8 us");
    end

    always @(posedge sck) begin : sample_transaction
        reg [7:0] spi_command;
        reg [7:0] qpi_command;
        if (!cs_n) begin
            if (!qpi_mode) begin
                if (host_oe[0] !== 1'b1) begin
                    report_error("SPI command not driven on SD0");
                    state = M_IDLE;
                end else if (state == M_COMMAND) begin
                    if (spi_bit_count == 3'd7) begin
                        spi_command = {spi_shift[6:0], host_sd[0]};
                        spi_shift = {spi_shift[6:0], host_sd[0]};
                        spi_bit_count = 3'd0;
                        if (spi_command == 8'h35) begin
                            qpi_mode = 1'b1;
                            state = M_IDLE;
                        end else if (spi_command == 8'hf5) begin
                            state = M_IDLE;
                        end else begin
                            $display("PSRAM ERROR: unknown SPI command 0x%02x at %0t",
                                     spi_command, $time);
                            error_count = error_count + 1'b1;
                            state = M_IDLE;
                        end
                    end else begin
                        spi_shift = {spi_shift[6:0], host_sd[0]};
                        spi_bit_count = spi_bit_count + 3'd1;
                    end
                end
            end else begin
                case (state)
                    M_COMMAND: begin
                        if (host_oe !== 4'b1111) begin
                            report_error("QPI command lines not driven");
                            state = M_IDLE;
                        end else begin
                            if (nibble_count == 3'd1) begin
                                qpi_command = {qpi_shift[3:0], host_sd};
                                qpi_shift = {qpi_shift[3:0], host_sd};
                                nibble_count = 3'd0;
                                if (qpi_command == 8'hf5) begin
                                    qpi_mode = 1'b0;
                                    state = M_IDLE;
                                end else if (qpi_command == 8'h0b) begin
                                    state = M_ADDR_READ;
                                    address_shift = 24'd0;
                                end else if ((qpi_command == 8'h02) ||
                                             (qpi_command == 8'h38)) begin
                                    state = M_ADDR_WR;
                                    address_shift = 24'd0;
                                end else begin
                                    report_error("unknown QPI command");
                                    state = M_IDLE;
                                end
                            end else begin
                                qpi_shift = {qpi_shift[3:0], host_sd};
                                nibble_count = 3'd1;
                            end
                        end
                    end

                    M_ADDR_READ, M_ADDR_WR: begin
                        if (host_oe !== 4'b1111) begin
                            report_error("QPI address lines not driven");
                            state = M_IDLE;
                        end else begin
                            address_shift = shifted_address;
                            if (nibble_count == 3'd5) begin
                                current_addr = shifted_address;
                                nibble_count = 3'd0;
                                if (shifted_address >= 24'h040000) begin
                                    report_error("address outside modeled memory");
                                    state = M_IDLE;
                                end else if (state == M_ADDR_READ) begin
                                    state = M_DUMMY;
                                    dummy_count = 3'd0;
                                end else begin
                                    state = M_WRITE;
                                    write_phase = 1'b0;
                                end
                            end else begin
                                nibble_count = nibble_count + 3'd1;
                            end
                        end
                    end

                    M_DUMMY: begin
                        if (host_oe !== 4'b0000) begin
                            report_error("SD lines driven during read dummy cycles");
                            state = M_IDLE;
                        end else if (dummy_count == 3'd3) begin
                            state = M_READ;
                            read_phase = 1'b0;
                        end else begin
                            dummy_count = dummy_count + 3'd1;
                        end
                    end

                    M_READ: begin
                        if (host_oe !== 4'b0000) begin
                            report_error("SD lines driven during read");
                            state = M_IDLE;
                        end
                    end

                    M_WRITE: begin
                        if (host_oe !== 4'b1111) begin
                            report_error("QPI write lines not driven");
                            state = M_IDLE;
                        end else if (!write_phase) begin
                            write_high = host_sd;
                            write_phase = 1'b1;
                        end else begin
                            if (current_addr >= 24'h040000) begin
                                report_error("write outside modeled memory");
                            end else begin
                                mem[current_addr] = {write_high, host_sd};
                            end
                            current_addr = {
                                current_addr[23:10],
                                current_addr[9:0] + 10'd1
                            };
                            write_phase = 1'b0;
                        end
                    end

                    default: begin
                        report_error("unexpected QPI state");
                        state = M_IDLE;
                    end
                endcase
            end
        end
    end

    always @(negedge sck) begin : drive_read_data
        if (!cs_n && (state == M_READ)) begin
            #(read_delay_ns);
            if (!cs_n && (state == M_READ)) begin
                if (current_addr >= 24'h040000) begin
                    report_error("read outside modeled memory");
                    dev_oe = 4'd0;
                end else begin
                    dev_oe = 4'b1111;
                    if (!read_phase) begin
                        dev_sd = mem[current_addr][7:4];
                        read_phase = 1'b1;
                    end else begin
                        dev_sd = mem[current_addr][3:0];
                        read_phase = 1'b0;
                        current_addr = {
                            current_addr[23:10],
                            current_addr[9:0] + 10'd1
                        };
                    end
                end
            end
        end
    end

endmodule

`default_nettype wire
