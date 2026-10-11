`default_nettype none

module lz4_engine (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire        mode,
    input  wire        push,
    input  wire [7:0]  push_data,
    input  wire        end_job,
    input  wire        pop,
    input  wire        psram_ready,
    input  wire        psram_busy,
    input  wire        ps_bdone,
    input  wire [7:0]  ps_rdata,
    output wire        ps_start,
    output reg         ps_we,
    output reg  [17:0] ps_addr,
    output reg  [7:0]  ps_wdata,
    output wire        ps_more,
    output wire        in_ready,
    output reg         out_valid,
    output wire        done,
    output reg         error,
    output reg  [7:0]  out_byte
);

    localparam [4:0] S_WAIT_START          = 5'd0;
    localparam [4:0] S_COMP_LOAD           = 5'd1;
    localparam [4:0] S_COMP_HASH           = 5'd2;
    localparam [4:0] S_COMP_CHECK_ENTRY    = 5'd3;
    localparam [4:0] S_COMP_CAND_CHECK     = 5'd4;
    localparam [4:0] S_COMP_EXT            = 5'd5;
    localparam [4:0] S_COMP_EXT_REF        = 5'd6;
    localparam [4:0] S_COMP_EXT_CHECK      = 5'd7;
    localparam [4:0] S_COMP_EMIT_TOKEN     = 5'd8;
    localparam [4:0] S_COMP_EMIT_LL_EXT    = 5'd9;
    localparam [4:0] S_COMP_EMIT_LITERAL   = 5'd10;
    localparam [4:0] S_COMP_EMIT_LITERAL_O = 5'd11;
    localparam [4:0] S_COMP_EMIT_OFFSET_L  = 5'd12;
    localparam [4:0] S_COMP_EMIT_OFFSET_H  = 5'd13;
    localparam [4:0] S_COMP_EMIT_ML_EXT    = 5'd14;
    localparam [4:0] S_COMP_EMIT_FINISH    = 5'd15;
    localparam [4:0] S_DEC_TOKEN           = 5'd16;
    localparam [4:0] S_DEC_LL_EXT          = 5'd17;
    localparam [4:0] S_DEC_LITERAL         = 5'd18;
    localparam [4:0] S_DEC_AFTER_LIT       = 5'd19;
    localparam [4:0] S_DEC_OFFSET_H        = 5'd21;
    localparam [4:0] S_DEC_ML_EXT          = 5'd22;
    localparam [4:0] S_DEC_COPY            = 5'd23;
    localparam [4:0] S_DEC_COPY_WRITE      = 5'd24;
    localparam [4:0] S_PS_ISSUE            = 5'd25;
    localparam [4:0] S_PS_WAIT             = 5'd26;
    localparam [4:0] S_DONE                = 5'd27;
    localparam [4:0] S_COMP_TABLE_WRITE    = 5'd28;

    localparam [3:0] OP_NONE        = 4'd0;
    localparam [3:0] OP_WRITE_INPUT = 4'd1;
    localparam [3:0] OP_READ_CUR    = 4'd2;
    localparam [3:0] OP_READ_TABLE  = 4'd3;
    localparam [3:0] OP_WRITE_TABLE = 4'd4;
    localparam [3:0] OP_READ_CAND   = 4'd5;
    localparam [3:0] OP_READ_EXT    = 4'd6;
    localparam [3:0] OP_READ_REF    = 4'd7;
    localparam [3:0] OP_READ_LITERAL = 4'd8;
    localparam [3:0] OP_WRITE_LITERAL = 4'd9;
    localparam [3:0] OP_READ_COPY   = 4'd10;
    localparam [3:0] OP_WRITE_COPY  = 4'd11;

    reg [4:0] state;
    reg [4:0] op_return;
    reg [3:0] op_kind;
    reg [2:0] op_len;
    reg [2:0] op_index;
    reg       op_mode;
    reg       started;
    reg       in_full;
    reg       end_flag;
    reg [7:0] ibyte;
    reg [15:0] w;
    reg [15:0] p;
    reg [15:0] anchor;
    reg [15:0] q;
    reg [15:0] e;
    reg [15:0] cnt;
    reg [15:0] emit_ml;
    reg [3:0]  match_nibble;
    reg [31:0] cur;
    reg        match_ok;
    reg [2:0]  prefix_len;
    reg        prefix_equal;
    reg [7:0]  write_byte;
    reg [17:0] op_addr;

    wire eof = end_flag && !in_full;
    wire [15:0] hash_value;
    wire [17:0] table_addr;
    wire [3:0] token_ll = (p >= 16'd15) ? 4'hf : p[3:0];
    wire [3:0] token_ml = (emit_ml >= 16'd19) ? 4'hf :
                          ((emit_ml >= 16'd4) ? (emit_ml[3:0] - 4'd4) : 4'd0);
    wire [16:0] extension_sum = {1'b0, cnt} + {9'd0, ibyte};

    function [15:0] lz4_hash;
        input [31:0] value;
        reg [15:0] mixed;
        begin
            mixed = value[15:0] ^ value[22:7] ^ value[28:13] ^
                    {5'd0, value[31:21]};
            lz4_hash = mixed;
        end
    endfunction

    assign hash_value = lz4_hash(cur);
    assign table_addr = {1'b1, hash_value, 1'b0};
    assign in_ready = started && !in_full && !end_flag && !done && !error;
    assign done = (state == S_DONE) && !out_valid;
    assign ps_start = (state == S_PS_ISSUE) && !psram_busy;
    assign ps_more = ({1'b0, op_index} + 4'd1) < {1'b0, op_len};

    always @* begin
        ps_we = op_mode;
        ps_addr = op_addr;
        ps_wdata = write_byte;
        if (op_kind == OP_WRITE_TABLE) begin
            if (op_index == 3'd0)
                ps_wdata = p[7:0];
            else
                ps_wdata = p[15:8];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_WAIT_START;
            op_return <= S_WAIT_START;
            op_kind <= OP_NONE;
            op_len <= 3'd0;
            op_index <= 3'd0;
            op_mode <= 1'b0;
            started <= 1'b0;
            in_full <= 1'b0;
            end_flag <= 1'b0;
            ibyte <= 8'd0;
            w <= 16'd0;
            p <= 16'd0;
            anchor <= 16'd0;
            q <= 16'd0;
            e <= 16'd0;
            cnt <= 16'd0;
            emit_ml <= 16'd0;
            match_nibble <= 4'd0;
            cur <= 32'd0;
            match_ok <= 1'b0;
            prefix_len <= 3'd0;
            prefix_equal <= 1'b0;
            write_byte <= 8'd0;
            op_addr <= 18'd0;
            out_valid <= 1'b0;
            error <= 1'b0;
            out_byte <= 8'd0;
        end else if (start && psram_ready && !psram_busy) begin
            state <= mode ? S_DEC_TOKEN : S_COMP_LOAD;
            op_kind <= OP_NONE;
            op_len <= 3'd0;
            op_index <= 3'd0;
            started <= 1'b1;
            in_full <= 1'b0;
            end_flag <= 1'b0;
            ibyte <= 8'd0;
            w <= 16'd0;
            p <= 16'd0;
            anchor <= 16'd0;
            q <= 16'd0;
            e <= 16'd0;
            cnt <= 16'd0;
            emit_ml <= 16'd0;
            match_nibble <= 4'd0;
            cur <= 32'd0;
            match_ok <= 1'b0;
            prefix_len <= 3'd0;
            prefix_equal <= 1'b0;
            write_byte <= 8'd0;
            out_valid <= 1'b0;
            error <= 1'b0;
            out_byte <= 8'd0;
        end else begin
            if (push) begin
                ibyte <= push_data;
                in_full <= 1'b1;
            end
            if (end_job)
                end_flag <= 1'b1;
            if (pop)
                out_valid <= 1'b0;

            case (state)
                S_WAIT_START: begin
                    state <= S_WAIT_START;
                end

                S_COMP_LOAD: begin
                    if (in_full) begin
                        op_kind <= OP_WRITE_INPUT;
                        op_mode <= 1'b1;
                        op_addr <= {2'd0, w};
                        op_len <= 3'd1;
                        op_index <= 3'd0;
                        op_return <= S_COMP_LOAD;
                        write_byte <= ibyte;
                        state <= S_PS_ISSUE;
                    end else if ((w - p) >= 16'd12) begin
                        op_kind <= OP_READ_CUR;
                        op_mode <= 1'b0;
                        op_addr <= {2'd0, p};
                        op_len <= 3'd4;
                        op_index <= 3'd0;
                        state <= S_PS_ISSUE;
                    end else if (eof) begin
                        p <= w - anchor;
                        emit_ml <= 16'd0;
                        state <= S_COMP_EMIT_TOKEN;
                    end
                end

                S_COMP_HASH: begin
                    op_kind <= OP_READ_TABLE;
                    op_mode <= 1'b0;
                        op_addr <= table_addr;
                    op_len <= 3'd2;
                    op_index <= 3'd0;
                    state <= S_PS_ISSUE;
                end

                S_COMP_TABLE_WRITE: begin
                    op_kind <= OP_WRITE_TABLE;
                    op_mode <= 1'b1;
                    op_addr <= table_addr;
                    op_len <= 3'd2;
                    op_index <= 3'd0;
                    state <= S_PS_ISSUE;
                end

                S_COMP_CHECK_ENTRY: begin
                    if (e < p) begin
                        match_ok <= 1'b1;
                        op_kind <= OP_READ_CAND;
                        op_mode <= 1'b0;
                        op_addr <= {2'd0, e};
                        op_len <= 3'd4;
                        op_index <= 3'd0;
                        state <= S_PS_ISSUE;
                    end else begin
                        p <= p + 16'd1;
                        state <= S_COMP_LOAD;
                    end
                end

                S_COMP_CAND_CHECK: begin
                    if (match_ok) begin
                        e <= p - e;
                        q <= p + 16'd4;
                        state <= S_COMP_EXT;
                    end else begin
                        p <= p + 16'd1;
                        state <= S_COMP_LOAD;
                    end
                end

                S_COMP_EXT: begin
                    if (in_full) begin
                        op_kind <= OP_WRITE_INPUT;
                        op_mode <= 1'b1;
                        op_addr <= {2'd0, w};
                        op_len <= 3'd1;
                        op_index <= 3'd0;
                        op_return <= S_COMP_EXT;
                        write_byte <= ibyte;
                        state <= S_PS_ISSUE;
                    end else if ((w - q) >= 16'd9) begin
                        op_kind <= OP_READ_EXT;
                        op_mode <= 1'b0;
                        op_addr <= {2'd0, q};
                        op_len <= 3'd4;
                        op_index <= 3'd0;
                        state <= S_PS_ISSUE;
                    end else if (eof) begin
                        p <= p - anchor;
                        emit_ml <= q - p;
                        state <= S_COMP_EMIT_TOKEN;
                    end
                end

                S_COMP_EXT_REF: begin
                    prefix_len <= 3'd0;
                    prefix_equal <= 1'b1;
                    op_kind <= OP_READ_REF;
                    op_mode <= 1'b0;
                    op_addr <= {2'd0, (q - e)};
                    op_len <= 3'd4;
                    op_index <= 3'd0;
                    state <= S_PS_ISSUE;
                end

                S_COMP_EXT_CHECK: begin
                    if (prefix_len == 3'd4) begin
                        q <= q + 16'd4;
                        state <= S_COMP_EXT;
                    end else begin
                        q <= q + {13'd0, prefix_len};
                        p <= p - anchor;
                        emit_ml <= (q + {13'd0, prefix_len}) - p;
                        state <= S_COMP_EMIT_TOKEN;
                    end
                end

                S_COMP_EMIT_TOKEN: begin
                    if (!out_valid) begin
                        out_byte <= {token_ll, token_ml};
                        out_valid <= 1'b1;
                        if (p >= 16'd15) begin
                            cnt <= p - 16'd15;
                            state <= S_COMP_EMIT_LL_EXT;
                        end else if (p != 16'd0) begin
                            cnt <= p;
                            state <= S_COMP_EMIT_LITERAL;
                        end else if (emit_ml != 16'd0) begin
                            state <= S_COMP_EMIT_OFFSET_L;
                        end else begin
                            state <= S_DONE;
                        end
                    end
                end

                S_COMP_EMIT_LL_EXT: begin
                    if (!out_valid) begin
                        if (cnt >= 16'd255) begin
                            out_byte <= 8'hff;
                            out_valid <= 1'b1;
                            cnt <= cnt - 16'd255;
                        end else begin
                            out_byte <= cnt[7:0];
                            out_valid <= 1'b1;
                            if (p != 16'd0) begin
                                cnt <= p;
                                state <= S_COMP_EMIT_LITERAL;
                            end else if (emit_ml != 16'd0)
                                state <= S_COMP_EMIT_OFFSET_L;
                            else
                                state <= S_DONE;
                        end
                    end
                end

                S_COMP_EMIT_LITERAL: begin
                    if (cnt == 16'd0) begin
                        if (emit_ml != 16'd0)
                            state <= S_COMP_EMIT_OFFSET_L;
                        else
                            state <= S_DONE;
                    end else if (!out_valid && !psram_busy) begin
                        op_kind <= OP_READ_LITERAL;
                        op_mode <= 1'b0;
                        op_addr <= {2'd0, anchor};
                        op_len <= 3'd1;
                        op_index <= 3'd0;
                        state <= S_PS_ISSUE;
                    end
                end

                S_COMP_EMIT_LITERAL_O: begin
                    if (!out_valid) begin
                        out_byte <= write_byte;
                        out_valid <= 1'b1;
                        anchor <= anchor + 16'd1;
                        cnt <= cnt - 16'd1;
                        state <= S_COMP_EMIT_LITERAL;
                    end
                end

                S_COMP_EMIT_OFFSET_L: begin
                    if (!out_valid) begin
                        out_byte <= e[7:0];
                        out_valid <= 1'b1;
                        state <= S_COMP_EMIT_OFFSET_H;
                    end
                end

                S_COMP_EMIT_OFFSET_H: begin
                    if (!out_valid) begin
                        out_byte <= e[15:8];
                        out_valid <= 1'b1;
                        if (emit_ml >= 16'd19) begin
                            cnt <= emit_ml - 16'd19;
                            state <= S_COMP_EMIT_ML_EXT;
                        end else begin
                            state <= S_COMP_EMIT_FINISH;
                        end
                    end
                end

                S_COMP_EMIT_ML_EXT: begin
                    if (!out_valid) begin
                        if (cnt >= 16'd255) begin
                            out_byte <= 8'hff;
                            out_valid <= 1'b1;
                            cnt <= cnt - 16'd255;
                        end else begin
                            out_byte <= cnt[7:0];
                            out_valid <= 1'b1;
                            cnt <= 16'd0;
                            state <= S_COMP_EMIT_FINISH;
                        end
                    end
                end

                S_COMP_EMIT_FINISH: begin
                    p <= q;
                    anchor <= q;
                    state <= S_COMP_LOAD;
                end

                S_DEC_TOKEN: begin
                    if (eof) begin
                        state <= S_DONE;
                    end else if (in_full) begin
                        in_full <= 1'b0;
                        match_nibble <= ibyte[3:0];
                        if (ibyte[7:4] == 4'hf) begin
                            cnt <= 16'd15;
                            state <= S_DEC_LL_EXT;
                        end else begin
                            cnt <= {12'd0, ibyte[7:4]};
                            if (ibyte[7:4] == 4'd0)
                                state <= S_DEC_AFTER_LIT;
                            else
                                state <= S_DEC_LITERAL;
                        end
                    end
                end

                S_DEC_LL_EXT: begin
                    if (eof) begin
                        error <= 1'b1;
                        state <= S_DONE;
                    end else if (in_full) begin
                        if (extension_sum[16]) begin
                            error <= 1'b1;
                            state <= S_DONE;
                        end else begin
                            in_full <= 1'b0;
                            cnt <= extension_sum[15:0];
                            if (ibyte != 8'hff) begin
                                if (extension_sum[15:0] == 16'd0)
                                    state <= S_DEC_AFTER_LIT;
                                else
                                    state <= S_DEC_LITERAL;
                            end
                        end
                    end
                end

                S_DEC_LITERAL: begin
                    if (cnt == 16'd0) begin
                        state <= S_DEC_AFTER_LIT;
                    end else if (eof) begin
                        error <= 1'b1;
                        state <= S_DONE;
                    end else if (w == 16'hffff) begin
                        error <= 1'b1;
                        state <= S_DONE;
                    end else if (in_full && !out_valid && !psram_busy) begin
                        write_byte <= ibyte;
                        op_kind <= OP_WRITE_LITERAL;
                        op_mode <= 1'b1;
                        op_addr <= {2'd0, w};
                        op_len <= 3'd1;
                        op_index <= 3'd0;
                        state <= S_PS_ISSUE;
                    end
                end

                S_DEC_AFTER_LIT: begin
                    if (eof) begin
                        state <= S_DONE;
                    end else if (in_full) begin
                        e[7:0] <= ibyte;
                        in_full <= 1'b0;
                        state <= S_DEC_OFFSET_H;
                    end
                end

                S_DEC_OFFSET_H: begin
                    if (eof) begin
                        error <= 1'b1;
                        state <= S_DONE;
                    end else if (in_full) begin
                        in_full <= 1'b0;
                        if (({ibyte, e[7:0]} == 16'd0) ||
                            ({ibyte, e[7:0]} > w)) begin
                            error <= 1'b1;
                            state <= S_DONE;
                        end else begin
                            e <= {ibyte, e[7:0]};
                            if (match_nibble == 4'hf) begin
                                cnt <= 16'd19;
                                state <= S_DEC_ML_EXT;
                            end else begin
                                cnt <= {12'd0, match_nibble} + 16'd4;
                                state <= S_DEC_COPY;
                            end
                        end
                    end
                end

                S_DEC_ML_EXT: begin
                    if (eof) begin
                        error <= 1'b1;
                        state <= S_DONE;
                    end else if (in_full) begin
                        if (extension_sum[16]) begin
                            error <= 1'b1;
                            state <= S_DONE;
                        end else begin
                            in_full <= 1'b0;
                            cnt <= extension_sum[15:0];
                            if (ibyte != 8'hff)
                                state <= S_DEC_COPY;
                        end
                    end
                end

                S_DEC_COPY: begin
                    if (cnt == 16'd0) begin
                        state <= S_DEC_TOKEN;
                    end else if (w == 16'hffff) begin
                        error <= 1'b1;
                        state <= S_DONE;
                    end else if (!out_valid && !psram_busy) begin
                        op_kind <= OP_READ_COPY;
                        op_mode <= 1'b0;
                        op_addr <= {2'd0, (w - e)};
                        op_len <= 3'd1;
                        op_index <= 3'd0;
                        state <= S_PS_ISSUE;
                    end
                end

                S_DEC_COPY_WRITE: begin
                    if (!psram_busy) begin
                        op_kind <= OP_WRITE_COPY;
                        op_mode <= 1'b1;
                        op_addr <= {2'd0, w};
                        op_len <= 3'd1;
                        op_index <= 3'd0;
                        state <= S_PS_ISSUE;
                    end
                end

                S_PS_ISSUE: begin
                    if (!psram_busy)
                        state <= S_PS_WAIT;
                end

                S_PS_WAIT: begin
                    if (ps_bdone) begin
                        case (op_kind)
                            OP_READ_CUR: begin
                                case (op_index)
                                    3'd0: cur[7:0] <= ps_rdata;
                                    3'd1: cur[15:8] <= ps_rdata;
                                    3'd2: cur[23:16] <= ps_rdata;
                                    default: cur[31:24] <= ps_rdata;
                                endcase
                            end
                            OP_READ_TABLE: begin
                                if (op_index == 3'd0)
                                    e[7:0] <= ps_rdata;
                                else
                                    e[15:8] <= ps_rdata;
                            end
                            OP_READ_CAND: begin
                                if (ps_rdata != cur[7:0])
                                    match_ok <= 1'b0;
                                cur <= {8'd0, cur[31:8]};
                            end
                            OP_READ_REF: begin
                                if (prefix_equal && (ps_rdata == cur[7:0]))
                                    prefix_len <= prefix_len + 3'd1;
                                else
                                    prefix_equal <= 1'b0;
                                cur <= {8'd0, cur[31:8]};
                            end
                            OP_READ_EXT: begin
                                case (op_index)
                                    3'd0: cur[7:0] <= ps_rdata;
                                    3'd1: cur[15:8] <= ps_rdata;
                                    3'd2: cur[23:16] <= ps_rdata;
                                    default: cur[31:24] <= ps_rdata;
                                endcase
                            end
                            OP_READ_LITERAL, OP_READ_COPY: begin
                                write_byte <= ps_rdata;
                            end
                            default: begin end
                        endcase

                        if (ps_more) begin
                            op_index <= op_index + 3'd1;
                        end else begin
                            case (op_kind)
                                OP_WRITE_INPUT: begin
                                    in_full <= 1'b0;
                                    w <= w + 16'd1;
                                    state <= op_return;
                                end
                                OP_READ_CUR: begin
                                    state <= S_COMP_HASH;
                                end
                                OP_READ_TABLE: begin
                                    state <= S_COMP_TABLE_WRITE;
                                end
                                OP_WRITE_TABLE: begin
                                    state <= S_COMP_CHECK_ENTRY;
                                end
                                OP_READ_CAND: begin
                                    state <= S_COMP_CAND_CHECK;
                                end
                                OP_READ_EXT: begin
                                    state <= S_COMP_EXT_REF;
                                end
                                OP_READ_REF: begin
                                    state <= S_COMP_EXT_CHECK;
                                end
                                OP_READ_LITERAL: begin
                                    state <= S_COMP_EMIT_LITERAL_O;
                                end
                                OP_WRITE_LITERAL: begin
                                    in_full <= 1'b0;
                                    w <= w + 16'd1;
                                    out_byte <= write_byte;
                                    out_valid <= 1'b1;
                                    cnt <= cnt - 16'd1;
                                    if (cnt == 16'd1)
                                        state <= S_DEC_AFTER_LIT;
                                    else
                                        state <= S_DEC_LITERAL;
                                end
                                OP_READ_COPY: begin
                                    state <= S_DEC_COPY_WRITE;
                                end
                                OP_WRITE_COPY: begin
                                    w <= w + 16'd1;
                                    out_byte <= write_byte;
                                    out_valid <= 1'b1;
                                    cnt <= cnt - 16'd1;
                                    if (cnt == 16'd1)
                                        state <= S_DEC_TOKEN;
                                    else
                                        state <= S_DEC_COPY;
                                end
                                default: begin
                                    state <= S_DONE;
                                end
                            endcase
                        end
                    end
                end

                S_DONE: begin
                    state <= S_DONE;
                end

                default: begin
                    state <= S_WAIT_START;
                    error <= 1'b1;
                end
            endcase
        end
    end

endmodule

`default_nettype wire
