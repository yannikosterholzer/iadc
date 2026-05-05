module analog_ctrl #(
    parameter int unsigned N_C      = 2069,    // total cycle length
    parameter int unsigned N_RU     = 1024,    // integration window
    parameter int unsigned N_DE_MAX = 1024,    // max deintegration
    parameter int unsigned T_R      = 8,       // recover-fixed length
    parameter int unsigned T_S      = 3        // non-overlap gap
) (
    input  logic        clk,
    input  logic        resetn,                // active-low async reset
    input  logic        vcomp,                 // comparator output (used directly, no sync FF)
    output logic        s1,
    output logic        s2a,
    output logic        s2b,
    output logic        s3,                    // discharges integration cap during REC_FIX
    output logic        dout_valid,            // output valid (cnt,sgn,ovf)
    output logic [9:0]  cnt_out,               // counter value at vcomp edge (or 0 on ovf)
    output logic        sgn_out,               // sign latched from vcomp during NONOV2
    output logic        ovf_out                // 1 = no edge during deint == counter saturated
);

    typedef enum logic [6:0] {
        REC_FIX     = 7'b00_0001_0,    // s3=1  (cap discharge)
        INTEGRATE   = 7'b00_1000_1,    // s1=1 + dout_valid=1
        DEINT_POS   = 7'b00_0010_0,    // s2b=1
        DEINT_NEG   = 7'b00_0100_0,    // s2a=1
        NONOV1      = 7'b00_0000_0,    // all switches off (disc=00)
        NONOV2      = 7'b01_0000_0,    //   "    "    "    (disc=01)
        NONOV3      = 7'b10_0000_0,    //   "    "    "    (disc=10)
        REC_VAR     = 7'b11_0000_0     //   "    "    "    (disc=11)
    } state_t;

    state_t       state, next;
    logic [10:0]  counter;
    logic [10:0]  deint_cnt_latched;
    logic [11:0]  rec_var_duration;
    logic         sgn_latched;


    assign {s1, s2a, s2b, s3, dout_valid} = {state[4:1], state[0]};
    assign cnt_out = deint_cnt_latched[9:0];
    assign ovf_out = deint_cnt_latched[10];
    assign sgn_out = sgn_latched;


    always_ff @(posedge clk or negedge resetn) begin : STATE_REG
        if (!resetn)     state <= REC_FIX;
        else             state <= next;
    end

    always_ff @(posedge clk or negedge resetn) begin : COUNTER_REG
        if (!resetn)
            counter <= 11'd0;
        else if (state != next)
            counter <= 11'd0;
        else
            counter <= counter + 11'd1;
    end

    assign rec_var_duration = (N_C > (T_R + N_RU + 3*T_S + deint_cnt_latched))? 12'(N_C - (T_R + N_RU + 3*T_S + deint_cnt_latched)): 12'd1;

    always_comb begin : TRANSITION_LOGIC
        next = state;
        unique case (state)
            REC_FIX:   if (counter == T_R  - 11'd1)             next = NONOV1;
            NONOV1:    if (counter == T_S  - 11'd1)             next = INTEGRATE;
            INTEGRATE: if (counter == N_RU - 11'd1)             next = NONOV2;
            NONOV2:    if (counter == T_S  - 11'd1)             next = vcomp ? DEINT_NEG : DEINT_POS;
            DEINT_POS: if (vcomp || counter[10])                next = NONOV3;
            DEINT_NEG: if (!vcomp || counter[10])               next = NONOV3;
            NONOV3:    if (counter == T_S - 11'd1)              next = REC_VAR;
            REC_VAR:   if (counter == rec_var_duration - 12'd1) next = REC_FIX;
            default:                                            next = REC_FIX;
        endcase
    end

    always_ff @(posedge clk or negedge resetn) begin : STATE_LATCHES
        if (!resetn) begin
            sgn_latched       <= 1'b0;
            deint_cnt_latched <= 11'd0;
        end
        else begin
            if (state == NONOV2 && (next == DEINT_POS || next == DEINT_NEG))
                sgn_latched <= !vcomp;
            if ((state == DEINT_POS || state == DEINT_NEG) && next == NONOV3)
                deint_cnt_latched <= counter;
        end
    end

endmodule
