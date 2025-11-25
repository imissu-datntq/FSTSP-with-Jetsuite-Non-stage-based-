#ifndef CFG_H
#define CFG_H
#include <set>
#include <string>

class Config {
public:
    // ---- Existing (kept) ----
    double dtl;   // Drone endurance E_d
    double sl;    // t_L (drone launch)
    double sr;    // t_R (drone recover)
    int    screen_mode; // 0: quiet, 1: solution only, 2: verbose
    bool   is_check;
    int    tl;          // time limit (sec), -1 => unlimited
    std::string output_path;
    int    test_mode;   // 0 normal, 1 not tighten, 2 z_{kk'}=0 if k'-k>=3

    // legacy switches (kept)

    int  L;
    int  num_thread;
    bool use_tsp_for_warmstart;
    bool use_cutting_plane;
    bool use_cplex_conflict_refiner;
    bool use_fstsp_for_warmstart;
    bool allow_loop;
    bool allow_multi_visit_drone;
    bool allow_revisit;

    // ---- JetSuite specific (NEW) ----
    double tJ_R; // JetSuite recover t_{JR} (existed but kept)
    double tJ_L; // JetSuite launch  t_{JL}  [NEW]

    // ---- LaTeX-required knobs (NEW) ----
    double ej;    // JetSuite endurance E_J   [NEW]
    double bigM;  // Big-M for time           [NEW]

    // ---- Optional knobs ----
    int    model_case; // 0: legacy, 1: Case1 (2-index), 2: Case2 (3-index no stage) [NEW]
    double w_wait;     // weight for ∑ T_i in objective (0 = ignore) [NEW]
    bool jetsuite_enabled;   // <-- thêm dòng này

    // ---- Ctors ----
    Config(int n_thread_val, double dtl_val, double sl_val, double sr_val, int L_val,
           bool use_tsp_ws_val, bool use_fstsp_for_ws_val, bool use_cutting_plane_val, bool use_conflict_refiner_val,
           bool allow_loop_val, bool allow_multi_visit_drone_val, bool allow_revisit_val, std::string &output_path_val)
    {
        num_thread = n_thread_val;
        dtl = dtl_val; sl = sl_val; sr = sr_val;
        L = L_val;
        use_tsp_for_warmstart = use_tsp_ws_val;
        use_cutting_plane = use_cutting_plane_val;
        use_cplex_conflict_refiner = use_conflict_refiner_val;
        use_fstsp_for_warmstart = use_fstsp_for_ws_val;
        allow_loop = allow_loop_val;
        allow_multi_visit_drone = allow_multi_visit_drone_val;
        allow_revisit = allow_revisit_val;

        screen_mode = 2; is_check = true; tl = -1;
        output_path = output_path_val; test_mode = 0;

        // NEW defaults
        tJ_R = sr; tJ_L = sl;
        ej   = dtl;
        bigM = 1e6;
        model_case = 0;
        w_wait = 0.0;
        jetsuite_enabled = false; // Mặc định tắt Jetsuite
    }

    Config (double dtl_val, double sl_val, double sr_val, int screen_mode_val, bool is_check_val, int tl_val,
            const std::string &output_path_val, bool allow_revisit_val, bool allow_loop_val,
            int n_thread_val, int test_mode_val)
    {
        dtl = dtl_val; sl = sl_val; sr = sr_val;
        screen_mode = screen_mode_val; is_check = is_check_val; tl = tl_val;
        output_path = output_path_val; allow_revisit = allow_revisit_val; allow_loop = allow_loop_val;
        num_thread = n_thread_val; test_mode = test_mode_val;

        // legacy init
        L = 1; use_tsp_for_warmstart = false; use_cutting_plane = false;
        use_cplex_conflict_refiner = false; use_fstsp_for_warmstart = false;
        allow_multi_visit_drone = false;

        // NEW defaults
        tJ_R = sr; tJ_L = sl;
        ej   = dtl; bigM = 1e6;
        model_case = 0; w_wait = 0.0;
        jetsuite_enabled = false; // Mặc định tắt Jetsuite
    }

    Config() {
        dtl = sl = sr = 0.0;
        screen_mode = 2; is_check = true; tl = -1;
        output_path = ""; test_mode = 0;
        L = 1; num_thread = 0;
        use_tsp_for_warmstart = false; use_cutting_plane = false;
        use_cplex_conflict_refiner = false; use_fstsp_for_warmstart = false;
        allow_loop = true; allow_multi_visit_drone = false; allow_revisit = false;

        // NEW defaults
        tJ_R = sr; tJ_L = sl;
        ej   = dtl; bigM = 1e6;
        model_case = 0; w_wait = 0.0;
        jetsuite_enabled = false; // Mặc định tắt Jetsuite

    }
};
#endif //CFG_H
