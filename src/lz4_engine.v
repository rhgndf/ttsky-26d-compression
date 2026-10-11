`default_nettype none

module lz4_engine (
    input wire clk, input wire rst_n, input wire start, input wire mode,
    input wire push, input wire [7:0] push_data, input wire end_job,
    input wire pop, input wire psram_ready, input wire psram_busy,
    input wire ps_bdone, input wire ps_nstrobe, input wire [3:0] ps_sd_in,
    output wire ps_start, output wire ps_we, output wire [17:0] ps_addr,
    output wire [3:0] ps_wnib, output wire ps_more, output wire in_ready,
    output reg out_valid, output wire done, output reg error,
    output reg [7:0] out_byte
);
    localparam [6:0]
      IDLE=0, LOAD=1, PTRP=2, PTRW=3, PTRWA=4, PTRWE=5, WR=6, WRA=7,
      WRE=8, RDCUR=9, HR=10, HW=11, CHECK=12, PTRE=13, CAND=14,
      CANDP=15, ADCHECK=16, ADP=17, ADPTR=18, ADV=19, EXCHECK=20,
      EXSP=21, EXS=22, EXTP=23, EXT=24, EXTPST=25, ELL=26, EML=27,
      EML4=28, LLP=29, MLP=30, TOK=31, LLEX=32, LITP=33, LITPTR=34,
      LITR=35, LITPOST=36, LITDEC=37, OFFL=38, OFFH=39, MLEX=40,
      FINP=41, FINA=42, FINALQ=43, DONE=44, DTOK=45, DLLEX=46,
      DLIT=47, DLITPTR=48, DLITWR=49, DLITPOST=50, DLITDEC=51,
      DAFTER=52, DOFFH=53, DCHECK=54, DBASE=55, DMLEX=56, DCOPY=57,
      DCOPYP=58, DCOPYR=59, DCOPYWP=60, DCOPYW=61, DCOPYPOST=62,
      DCOPYDEC=63, MATCHQ=64, ADVPOST=65, CURPOST=66, WRPOST=67,
      WRAPOST=68, WREPOST=69, MLEX_INIT=70, MLEX_BASE=71;
    localparam [6:0] LITANCH=72;

    reg [6:0] state;
    reg [15:0] w, p, q, e, anchor, ptr;
    reg [31:0] cur;
    reg [7:0] ibyte;
    reg in_full, started, end_flag;
    reg [2:0] nib;
    reg nib_phase, ok, has_match, final_block, ll_long, ml_long;

    reg [15:0] alu_a, alu_b;
    reg alu_inv, alu_cin;
    wire [16:0] alu_sum = {1'b0,alu_a} +
        {1'b0,(alu_b ^ {16{alu_inv}})} + alu_cin;
    wire [15:0] R = alu_sum[15:0];
    wire Rcarry = alu_sum[16];
    wire eof = end_flag && !in_full;
    wire [15:0] hash = cur[15:0] ^ cur[22:7] ^ cur[28:13] ^
                       {5'd0,cur[31:21]};
    wire table_access = state == HR || state == HW;
    wire write_state = state == WR || state == WRA || state == WRE ||
                       state == HW || state == DLITWR || state == DCOPYW;
    wire history_state = state == RDCUR || state == CAND || state == ADV ||
                         state == EXS || state == EXT || state == LITR ||
                         state == DCOPYR || state == WR || state == WRA ||
                         state == WRE || state == DLITWR || state == DCOPYW;
    wire byte_equal = ok && (ps_sd_in == cur[3:0]);

    always @* begin
        alu_a=0; alu_b=0; alu_inv=0; alu_cin=0;
        case (state)
          LOAD: begin alu_a=w; alu_b=p; alu_inv=1; alu_cin=1; end
          ADCHECK,EXCHECK: begin alu_a=w; alu_b=q; alu_inv=1; alu_cin=1; end
          PTRP: alu_a=p;
          PTRW,PTRWA,PTRWE: alu_a=w;
          WRA,WRE,ADVPOST,CURPOST,WRPOST,WRAPOST,WREPOST,EXTPST,
          LITPOST,DLITPOST,DCOPYPOST,LITANCH: alu_a=ptr;
          FINP,FINA: alu_a=q;
          LITPTR: alu_a=anchor;
          ADPTR,EXTP,DLITPTR,DCOPYWP: alu_a=q;
          CHECK,CANDP: begin alu_a=p; alu_b=e; alu_inv=1; alu_cin=1; end
          PTRE: alu_a=e;
          ADP: begin alu_a=p; alu_b=1; end
          MATCHQ: begin alu_a=p; alu_b=4; end
          EXSP: begin alu_a=q; alu_b=e; alu_inv=1; alu_cin=1; end
          ELL,LITP: begin alu_a=final_block ? w : p; alu_b=anchor;
                     alu_inv=1; alu_cin=1; end
          EML: begin alu_a=q; alu_b=p; alu_inv=1; alu_cin=1; end
          EML4: begin alu_a=cur[15:0]; alu_b=4; alu_inv=1; alu_cin=1; end
          LLP: begin alu_a=cur[31:16]; alu_b=15; alu_inv=1; alu_cin=1; end
          MLP: begin alu_a=cur[15:0]; alu_b=15; alu_inv=1; alu_cin=1; end
          MLEX_INIT: begin alu_a=q; alu_b=p; alu_inv=1; alu_cin=1; end
          MLEX_BASE: begin alu_a=cur[15:0]; alu_b=19;
                     alu_inv=1; alu_cin=1; end
          LLEX,MLEX: begin alu_a=(state==LLEX)?cur[31:16]:cur[15:0];
                     alu_b=255; alu_inv=1; alu_cin=1; end
          FINALQ: alu_a=w;
          DLLEX,DMLEX: begin alu_a=cur[15:0]; alu_b={8'd0,ibyte}; end
          DCHECK: begin alu_a=q; alu_b=e; alu_inv=1; alu_cin=1; end
          DBASE: begin alu_a={12'd0,p[3:0]}; alu_b=4; end
          DCOPYP: begin alu_a=q; alu_b=e; alu_inv=1; alu_cin=1; end
          LITDEC,DLITDEC,DCOPYDEC: begin alu_a=cur[15:0]; alu_b=1;
                                   alu_inv=1; alu_cin=1; end
          default: begin end
        endcase
    end

    assign ps_start = !psram_busy &&
      (state==WR || state==WRA || state==WRE || state==RDCUR ||
       state==HR || state==HW || state==CAND || state==ADV ||
       state==EXS || state==EXT || state==LITR || state==DLITWR ||
       state==DCOPYR || state==DCOPYW);
    assign ps_we = write_state;
    assign ps_addr = table_access ? {1'b1,hash,1'b0} : {2'b00,ptr};
    assign ps_more =
      (state==RDCUR || state==CAND || state==EXS) ? (nib != 3) :
      (state==HR || state==HW) ? (nib != 1) :
      (state==EXT) ? ((nib != 3) && byte_equal) : 1'b0;
    assign ps_wnib = (state==HW) ?
      (nib==0 ? (nib_phase ? p[7:4] : p[3:0]) :
                (nib_phase ? p[15:12] : p[11:8])) :
      ((state==WR || state==WRA || state==WRE || state==DLITWR) ?
        (nib_phase ? ibyte[7:4] : ibyte[3:0]) :
        (nib_phase ? out_byte[7:4] : out_byte[3:0]));
    assign in_ready = started && !in_full && !end_flag && !done && !error;
    assign done = (state==DONE) && !out_valid;

    always @(posedge clk or negedge rst_n) begin
      if (!rst_n) begin
        state<=IDLE; started<=0; in_full<=0; end_flag<=0;
        out_valid<=0; error<=0; nib<=0; nib_phase<=0; ok<=0;
        has_match<=0; final_block<=0; ll_long<=0; ml_long<=0;
      end else if (start && psram_ready && !psram_busy) begin
        state<=mode ? DTOK : LOAD; started<=1;
        in_full<=0; end_flag<=0; out_valid<=0; error<=0;
        w<=0; p<=0; q<=0; e<=0; anchor<=0; ptr<=0; nib<=0;
        nib_phase<=0; ok<=0; has_match<=0; final_block<=0;
        ll_long<=0; ml_long<=0;
      end else begin
        if (push) begin ibyte<=push_data; in_full<=1; end
        if (end_job) end_flag<=1;
        if (pop) out_valid<=0;

        if (ps_nstrobe) begin
          if (write_state) nib_phase<=~nib_phase;
          else if (state==RDCUR || state==ADV || state==EXS)
            cur<={ps_sd_in,cur[31:4]};
          else if (state==HR) e<={ps_sd_in,e[15:4]};
          else if (state==CAND || state==EXT) begin
            if (ps_sd_in != cur[3:0]) ok<=0;
            cur<={cur[3:0],cur[31:4]};
          end else if (state==LITR || state==DCOPYR)
            out_byte<={ps_sd_in,out_byte[7:4]};
        end
        if (ps_bdone && history_state &&
            ((state != EXT) || byte_equal))
          ptr<=ptr+16'd1;

        case (state)
          IDLE: begin end
          LOAD: begin
            if (in_full) state<=PTRW;
            else if (Rcarry && ((R[15:4]!=0)||(R[3]&&R[2])))
              state<=PTRP;
            else if (eof) begin final_block<=1; state<=FINALQ; end
          end
          PTRP: begin ptr<=R; nib<=0; nib_phase<=0; state<=RDCUR; end
          PTRW: begin ptr<=R; nib_phase<=0; state<=WR; end
          PTRWA: begin ptr<=R; nib_phase<=0; state<=WRA; end
          PTRWE: begin ptr<=R; nib_phase<=0; state<=WRE; end
          WR: if (ps_bdone) state<=WRPOST;
          WRA: if (ps_bdone) state<=WRAPOST;
          WRE: if (ps_bdone) state<=WREPOST;
          WRPOST: begin w<=R; in_full<=0; state<=LOAD; end
          WRAPOST: begin w<=R; in_full<=0; state<=ADCHECK; end
          WREPOST: begin w<=R; in_full<=0; state<=EXCHECK; end
          RDCUR: if (ps_bdone) begin
            if (nib==3) begin nib<=0; state<=CURPOST; end else nib<=nib+1;
          end
          CURPOST: begin q<=R; state<=HR; end
          HR: if (ps_bdone) begin
            if (nib==1) begin nib<=0; nib_phase<=0; state<=HW; end
            else nib<=nib+1;
          end
          HW: if (ps_bdone) begin
            if (nib==1) begin nib<=0; state<=CHECK; end else nib<=nib+1;
          end
          CHECK: begin
            if (Rcarry && R!=0) state<=PTRE; else state<=ADCHECK;
          end
          PTRE: begin ptr<=R; nib<=0; ok<=1; state<=CAND; end
          CAND: if (ps_bdone) begin
            if (nib==3) state<=CANDP; else nib<=nib+1;
          end
          CANDP: begin
            if (ok) begin e<=R; has_match<=1; state<=MATCHQ; end
            else state<=ADCHECK;
          end
          MATCHQ: begin q<=R; state<=EXCHECK; end
          ADCHECK: begin
            if (in_full) state<=PTRWA;
            else if (Rcarry && ((R[15:4]!=0)||(R[3]&&(R[2]||R[1]||R[0]))))
              state<=ADP;
            else if (eof) begin final_block<=1; state<=FINALQ; end
          end
          ADP: begin p<=R; state<=ADPTR; end
          ADPTR: begin ptr<=R; nib<=0; state<=ADV; end
          ADV: if (ps_bdone) state<=ADVPOST;
          ADVPOST: begin q<=R; state<=HR; end
          EXCHECK: begin
            if (in_full) state<=PTRWE;
            else if (Rcarry && ((R[15:4]!=0)||(R[3]&&(R[2]||R[1]||R[0]))))
              state<=EXSP;
            else if (eof) state<=ELL;
          end
          EXSP: begin ptr<=R; nib<=0; state<=EXS; end
          EXS: if (ps_bdone) begin
            if (nib==3) begin nib<=0; state<=EXTP; end else nib<=nib+1;
          end
          EXTP: begin ptr<=R; nib<=0; ok<=1; state<=EXT; end
          EXT: if (ps_bdone) begin
            if (!ps_more) state<=EXTPST; else nib<=nib+1;
          end
          EXTPST: begin q<=R; state<=ok ? EXCHECK : ELL; end
          ELL: begin cur[31:16]<=R; state<=EML; end
          EML: begin
            if (has_match) begin cur[15:0]<=R; state<=EML4; end
            else begin cur[15:0]<=0; state<=LLP; end
          end
          EML4: begin cur[15:0]<=R; state<=LLP; end
          LLP: begin
            if (Rcarry) begin cur[31:16]<=R; ll_long<=1; end
            else ll_long<=0;
            state<=has_match ? MLP : TOK;
          end
          MLP: begin
            if (Rcarry) begin cur[15:0]<=R; ml_long<=1; end
            else ml_long<=0;
            state<=TOK;
          end
          TOK: if (!out_valid) begin
            out_byte<={(ll_long?4'hf:cur[19:16]),
                       (has_match?(ml_long?4'hf:cur[3:0]):4'd0)};
            out_valid<=1; state<=ll_long ? LLEX : LITP;
          end
          LLEX: if (!out_valid) begin
            if (Rcarry) begin out_byte<=8'hff; out_valid<=1; cur[31:16]<=R; end
            else begin out_byte<=cur[23:16]; out_valid<=1; state<=LITP; end
          end
          LITP: if (!out_valid) begin
            cur[15:0]<=R;
            state<=R==0 ? (has_match?OFFL:DONE) : LITPTR;
          end
          LITPTR: begin ptr<=R; nib_phase<=0; state<=LITR; end
          LITR: if (ps_bdone) state<=LITPOST;
          LITPOST: if (!out_valid) begin out_valid<=1; state<=LITDEC; end
          LITDEC: if (!out_valid) begin
            cur[15:0]<=R; state<=LITANCH;
          end
          LITANCH: begin
            anchor<=R;
            state<=cur[15:0]==0 ? (has_match?OFFL:DONE) : LITPTR;
          end
          OFFL: if (!out_valid) begin out_byte<=e[7:0]; out_valid<=1; state<=OFFH; end
          OFFH: if (!out_valid) begin
            out_byte<=e[15:8]; out_valid<=1;
            state<=ml_long?MLEX_INIT:FINP;
          end
          MLEX_INIT: begin cur[15:0]<=R; state<=MLEX_BASE; end
          MLEX_BASE: begin cur[15:0]<=R; state<=MLEX; end
          MLEX: if (!out_valid) begin
            if (Rcarry) begin out_byte<=8'hff; out_valid<=1; cur[15:0]<=R; end
            else begin out_byte<=cur[7:0]; out_valid<=1; state<=FINP; end
          end
          FINP: begin p<=R; state<=FINA; end
          FINA: begin anchor<=R; has_match<=0; final_block<=0; state<=LOAD; end
          FINALQ: begin q<=R; has_match<=0; state<=ELL; end

          DTOK: begin
            if (eof) state<=DONE;
            else if (in_full) begin
              in_full<=0; p[3:0]<=ibyte[3:0];
              cur[15:0]<={12'd0,ibyte[7:4]};
              state<=ibyte[7:4]==15 ? DLLEX :
                     (ibyte[7:4]==0 ? DAFTER : DLIT);
            end
          end
          DLLEX: if (eof) begin error<=1; state<=DONE; end
            else if (in_full) begin
              if (Rcarry) begin error<=1; state<=DONE; end
              else begin in_full<=0; cur[15:0]<=R;
                if (ibyte!=8'hff) state<=R==0?DAFTER:DLIT;
              end
            end
          DLIT: begin
            if (cur[15:0]==0) state<=DAFTER;
            else if (eof) begin error<=1; state<=DONE; end
            else if (in_full && !out_valid) state<=DLITPTR;
          end
          DLITPTR: begin ptr<=R; nib_phase<=0; state<=DLITWR; end
          DLITWR: if (ps_bdone) state<=DLITPOST;
          DLITPOST: if (!out_valid) begin
            q<=R; out_byte<=ibyte; out_valid<=1; in_full<=0; state<=DLITDEC;
          end
          DLITDEC: begin cur[15:0]<=R; state<=R==0?DAFTER:DLIT; end
          DAFTER: begin
            if (eof) state<=DONE;
            else if (in_full) begin e[7:0]<=ibyte; in_full<=0; state<=DOFFH; end
          end
          DOFFH: if (eof) begin error<=1; state<=DONE; end
            else if (in_full) begin e[15:8]<=ibyte; in_full<=0; state<=DCHECK; end
          DCHECK: begin
            if (e==0 || !Rcarry) begin error<=1; state<=DONE; end
            else if (p[3:0]==15) begin cur[15:0]<=19; state<=DMLEX; end
            else state<=DBASE;
          end
          DBASE: begin cur[15:0]<=R; state<=DCOPY; end
          DMLEX: if (eof) begin error<=1; state<=DONE; end
            else if (in_full) begin
              if (Rcarry) begin error<=1; state<=DONE; end
              else begin in_full<=0; cur[15:0]<=R;
                if (ibyte!=8'hff) state<=DCOPY;
              end
            end
          DCOPY: begin
            if (cur[15:0]==0) state<=DTOK;
            else if (q==16'hffff) begin error<=1; state<=DONE; end
            else if (!out_valid) state<=DCOPYP;
          end
          DCOPYP: begin ptr<=R; nib_phase<=0; state<=DCOPYR; end
          DCOPYR: if (ps_bdone) state<=DCOPYWP;
          DCOPYWP: begin ptr<=R; nib_phase<=0; state<=DCOPYW; end
          DCOPYW: if (ps_bdone) state<=DCOPYPOST;
          DCOPYPOST: if (!out_valid) begin
            q<=R; out_valid<=1; state<=DCOPYDEC;
          end
          DCOPYDEC: begin cur[15:0]<=R; state<=R==0?DTOK:DCOPY; end
          DONE: begin end
          default: begin error<=1; state<=DONE; end
        endcase
      end
    end
endmodule

`default_nettype wire
