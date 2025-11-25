#include "../include/fstsp.h"
#include "../include/common.h"
#include "../include/solution.h"
#include <filesystem>
#include <chrono>
#include <ilcplex/ilocplex.h>
#include <set>
#include <unordered_map>
#include <unordered_set>

Result FSTSPSolver::RV_FSTSP_3_index(Config &cfg) const
{
    /**
     * Note in the implementation of the model (Output will be the same intend as Overleaf):
     * - The index of the nodes starts from 0 to N+1 (N+2 nodes in total)
     * - The index of the stages starts from 0 to K-1 (K stages in total)
     * - The index of the customers starts from 0 to N-1 (N customers in total)
     */
    IloEnv env;
    try
    {
        // Parameters
        int N = instance->num_node - 1; // Number of customers
        int S = 0;                      // Starting depot (node 0)
        int E = N + 1;                  // Ending depot (node N+1)
        int K = N + 2;                  // Number of possible stages for truck
        double D_d = cfg.dtl;           // Maximum flight time of the drone
        double t_L = cfg.sl;            // Time needed to launch drone
        double t_R = cfg.sr;            // Time needed to recover drone
        double tJ_L = cfg.sl;           // Time needed to launch jetsuit
        double tJ_R = cfg.sr;           // Time needed to recover jetsuit
        // Define set variables for the model. Make the code way easier to read

        std::set<int> C, V, V_minus_E, V_minus_S;
        for (int i = 1; i <= N; ++i)
        {
            C.insert(i);
        }
        V = C;
        V.insert(S);
        V_minus_E = V;
        V.insert(E);
        V_minus_S = C;
        V_minus_S.insert(E);

        // just for testing
        //        std::fill(instance->tau_prime.begin(), instance->tau_prime.end(), std::vector<double, std::allocator<double>>(N + 2, 0));

        // Travel times (2D float arrays)
        vector<vector<double>> tau = instance->tau;
        vector<vector<double>> tau_prime = instance->tau_prime;
        vector<vector<double>> tau_prime_prime = instance->tau_prime_prime;
        // Define the model
        IloModel model(env);

        // Define variables
        // X[i][k] binary variables
        IloArray<IloBoolVarArray> X(env, N + 2);
        for (int i : V)
        {
            X[i] = IloBoolVarArray(env, K + 1);
            for (int k = 1; k <= K; ++k)
            {
                std::stringstream varName;
                varName << "X_" << i << "_" << k;
                X[i][k] = IloBoolVar(env, 0, 1, varName.str().c_str());
            }
        }

        // x[i][j][k] binary variables
        IloArray<IloArray<IloBoolVarArray>> x(env, N + 2);
        for (int i : V)
        {
            x[i] = IloArray<IloBoolVarArray>(env, N + 2);
            for (int j : V)
            {
                x[i][j] = IloBoolVarArray(env, K + 1);
                for (int k = 1; k <= K; ++k)
                {
                    std::stringstream varName;
                    varName << "x_" << i << "_" << j << "_" << k;
                    x[i][j][k] = IloBoolVar(env, 0, 1, varName.str().c_str());
                }
            }
        }

        // phi[h] binary variables
        IloBoolVarArray phi(env, N + 1);
        for (int h : C)
        {
            std::stringstream varName;
            varName << "phi_" << h;
            phi[h] = IloBoolVar(env, 0, 1, varName.str().c_str());
        }

        // Y[i][h][k] binary variables
        IloArray<IloArray<IloBoolVarArray>> Y(env, N + 2);
        for (int i : V)
        {
            Y[i] = IloArray<IloBoolVarArray>(env, N + 1);
            for (int h : C)
            {
                Y[i][h] = IloBoolVarArray(env, K + 1);
                for (int k = 1; k <= K; ++k)
                {
                    std::stringstream varName;
                    varName << "Y_" << i << "_" << h << "_" << k;
                    Y[i][h][k] = IloBoolVar(env, 0, 1, varName.str().c_str());
                }
            }
        }

        // W[h][j][k] binary variables
        IloArray<IloArray<IloBoolVarArray>> W(env, N + 1);
        for (int h : C)
        {
            W[h] = IloArray<IloBoolVarArray>(env, N + 2);
            for (int j : V)
            {
                W[h][j] = IloBoolVarArray(env, K + 1);
                for (int k = 1; k <= K; ++k)
                {
                    std::stringstream varName;
                    varName << "W_" << h << "_" << j << "_" << k;
                    W[h][j][k] = IloBoolVar(env, 0, 1, varName.str().c_str());
                }
            }
        }

        // Zkk[k][k1] binary variables
        IloArray<IloBoolVarArray> Zkk(env, K + 1);
        for (int k = 1; k <= K; ++k)
        {
            Zkk[k] = IloBoolVarArray(env, K + 1);
            for (int k1 = 1; k1 <= K; ++k1)
            {
                std::stringstream varName;
                varName << "Zkk_" << k << "_" << k1;
                Zkk[k][k1] = IloBoolVar(env, 0, 1, varName.str().c_str());
            }
        }

        // Zhkk[h][k][k1] binary variables
        IloArray<IloArray<IloBoolVarArray>> Zhkk(env, N + 1);
        for (int h : C)
        {
            Zhkk[h] = IloArray<IloBoolVarArray>(env, K + 1);
            for (int k = 1; k <= K; ++k)
            {
                Zhkk[h][k] = IloBoolVarArray(env, K + 1);
                for (int k1 = 1; k1 <= K; ++k1)
                {
                    std::stringstream varName;
                    varName << "Zhkk_" << h << "_" << k << "_" << k1;
                    Zhkk[h][k][k1] = IloBoolVar(env, 0, 1, varName.str().c_str());
                }
            }
        }

        // a[k] continuous variables
        IloNumVarArray a(env, K + 1, 0, IloInfinity, ILOFLOAT);
        for (int k = 1; k <= K; ++k)
        {
            std::stringstream varName;
            varName << "a_" << k;
            a[k].setName(varName.str().c_str());
        }

        // d[k] continuous variables
        IloNumVarArray d(env, K + 1, 0, IloInfinity, ILOFLOAT);
        for (int k = 1; k <= K; ++k)
        {
            std::stringstream varName;
            varName << "d_" << k;
            d[k].setName(varName.str().c_str());
        }

        // Jetsuit option

        // phiJetsuite[h] binary variables
        IloBoolVarArray phiJetsuite(env, N + 1);
        for (int h : C)
        {
            std::stringstream varName;
            varName << "phiJ_" << h;
            phiJetsuite[h].setName(varName.str().c_str());
        }

        // U[i][h][k]  binary variables
        IloArray<IloArray<IloBoolVarArray>> U(env, N + 2);
        for (int i : V)
        {
            U[i] = IloArray<IloBoolVarArray>(env, N + 1);
            for (int h : C)
            {
                U[i][h] = IloBoolVarArray(env, K + 1);
                for (int k = 1; k <= K; ++k)
                {
                    std::stringstream varName;
                    varName << "U_" << i << "_" << h << "_" << k;
                    U[i][h][k].setName(varName.str().c_str());
                }
            }
        }
        // Objective function (minimize the complete time of the delivery route)
        IloExpr obj(env);
        obj += d[K];
        model.add(IloMinimize(env, obj));

        // Constraints
        // Constraint (1): Forbid visiting certain nodes, based on given instance
        for (int h : instance->no_visit)
        {
            model.add(phi[h] == 0).setName((string("C1_") + "h" + std::to_string(h)).c_str());
        }
        // Constraint (2): Truck can only at depot S or E in 1 stage only
        for (int i : {S, E})
        {
            IloExpr expr2(env);
            for (int k = 1; k <= K; ++k)
                expr2 += X[i][k];
            model.add(expr2 == 1).setName((string("C2_") + "i" + std::to_string(i)).c_str());
            expr2.end();
        }
        // Constraint (3): Truck starts at depot S
        model.add(X[S][1] == 1).setName("C3_a"); // X^1_S = 1
        IloExpr expr3(env);
        for (int j : V_minus_S)
            expr3 += x[S][j][1];
        model.add(expr3 == 1).setName("C3_b"); // Sum over outgoing edges from S
        expr3.end();
        // Constraint (4-5): Truck ends at depot E
        IloExpr expr4(env);
        IloExpr expr5(env);

        for (int k = 1; k <= K; ++k)
        {
            for (int i : V)
            {
                expr4 += x[i][E][k];
                expr5 += x[E][i][k];
            }
        }
        model.add(expr4 == 1).setName("C4");
        model.add(expr5 == 0).setName("C5");
        expr4.end();
        expr5.end();
        // Constraint (6): Outgoing edge if truck is at node i
        for (int k = 1; k <= K; ++k)
        {
            for (int i : V_minus_E)
            {
                IloExpr expr7(env);
                for (int j : V_minus_S)
                {
                    expr7 += x[i][j][k];
                }
                model.add(X[i][k] == expr7).setName((string("C6_") + "i" + std::to_string(i) + "_k" + std::to_string(k)).c_str());
                expr7.end();
            }
        }
        // Constraint (7): If the truck is at node i at position k > 1, there must be an incoming arc
        for (int k = 2; k <= K; ++k)
        {
            for (int i : V_minus_S)
            {
                IloExpr expr7_2(env);
                for (int j : V_minus_E)
                {
                    expr7_2 += x[j][i][k - 1];
                }
                model.add(X[i][k] == expr7_2).setName((string("C7_") + "i" + std::to_string(i) + "_k" + std::to_string(k)).c_str());
                expr7_2.end();
            }
        }

        //        for (int k = 1; k <= K; ++k) {
        //            IloExpr expr10(env);
        //            for (int i : V) {
        //                for (int j : V) {
        //                    expr10 += x[i][j][k];
        //                }
        //            }
        //            model.add(expr10 <= 1);
        //            expr10.end();
        //        }
        // Constraint (8): Only one RV_FSTSP_3_index can be visited at one stage
        for (int k = 1; k <= K; ++k)
        {
            IloExpr expr8(env);
            for (int i : V)
            {
                expr8 += X[i][k];
            }
            model.add(expr8 <= 1).setName((string("C8_") + "k" + std::to_string(k)).c_str());
            expr8.end();
        }
        // Constraint (13): Non-overlapping sorties
        for (int m = 1; m <= K - 1; m++)
        {
            IloExpr expr13(env);
            for (int k = 1; k <= m; k++)
                for (int k1 = m + 1; k1 <= K; k1++)
                    expr13 += Zkk[k][k1];
            model.add(expr13 <= 1).setName((string("C13_") + "m" + std::to_string(m)).c_str());
            expr13.end();
        }
        // Constraint (14): Each customer visited by drone or truck or jetsuite
        for (int h : C)
        {
            IloExpr expr14(env);
            for (int k = 1; k <= K; ++k)
                expr14 += X[h][k];
            model.add(phi[h] + phiJetsuite[h] + expr14 >= 1).setName((string("C14_") + "h" + std::to_string(h)).c_str());
            expr14.end();
        }
        // Constraint (16): Truck arrival and departure time at depot is 0
        model.add(a[1] == 0).setName("C16_a");
        model.add(d[1] == 0).setName("C16_b");
        // Constraint (17): Truck can only depart after recovering and/or launching the drone
        for (int k = 1; k <= K; ++k)
        {
            IloExpr expr17(env);
            for (int k1 = 1; k1 <= K; ++k1)
            {
                expr17 += Zkk[k1][k] * t_R + Zkk[k][k1] * t_L;
            }
            model.add(d[k] >= a[k] + expr17).setName((string("C17_") + "k" + std::to_string(k)).c_str());
            expr17.end();
        }
        // Constraint (18): Truck's travel time between consecutive positions
        for (int k = 1; k <= K - 1; ++k)
        {
            IloExpr expr18(env);
            for (int i : V)
            {
                for (int j : V)
                {
                    expr18 += x[i][j][k] * tau[i][j];
                }
            }
            model.add(a[k + 1] == d[k] + expr18).setName((string("C18_") + "k" + std::to_string(k)).c_str());
            expr18.end();
        }
        // Constraint (19): Truck-drone time's consistency 1
        for (int k = 1; k <= K - 1; ++k)
        {
            for (int k1 = k + 1; k1 <= K; ++k1)
            {
                IloExpr expr19(env);
                IloNum M = 1000; // A large number for big-M formulation
                expr19 += d[k] + Zkk[k][k1] * (D_d - t_R) + (1 - Zkk[k][k1]) * M;
                model.add(a[k1] <= expr19).setName((string("C19_") + "k" + std::to_string(k) + "_k'" + std::to_string(k1)).c_str());
                expr19.end();
            }
        }
        // Constraints (25, 28): Domain constraint
        for (int k = 1; k <= K; ++k)
        { // 25
            for (int k1 = 1; k1 <= k; ++k1)
            {
                model.add(Zkk[k][k1] ==
                          0)
                    .setName((string("C25_") + "k" + std::to_string(k) + "_k'" + std::to_string(k1)).c_str());
                for (int h : C) // Recently added
                    model.add(Zhkk[h][k][k1] ==
                              0)
                        .setName((string("C?1_") + "h" + std::to_string(h) + "_k" + std::to_string(k) + "_k'" + std::to_string(k1)).c_str());
            }
        }
        for (int k = 1; k <= K; k++)
        { // 28
            model.add(x[E][E][k] == 0).setName((string("C28E_") + "k" + std::to_string(k)).c_str());
            model.add(x[S][S][k] == 0).setName((string("C28S_") + "k" + std::to_string(k)).c_str());
        }
        for (int i : C)
            for (int k = 1; k <= K; k++)
            { // Recently added
                int h = i;
                model.add(Y[i][h][k] == 0).setName((string("C?2_") + "i" + std::to_string(i) + "_h" + std::to_string(h) + "_k" + std::to_string(k)).c_str());
                model.add(W[h][i][k] == 0).setName((string("C?3_") + "h" + std::to_string(h) + "_i" + std::to_string(i) + "_k" + std::to_string(k)).c_str());
            }
        // Strengthen model part
        if (cfg.test_mode != 1)
        {
            // Constraint (21): A customer is visited by either truck or drone, not both
            for (int i : C)
            {
                IloExpr expr21(env);
                for (int k = 1; k <= K; ++k)
                {
                    expr21 += X[i][k];
                }
                model.add(expr21 <= K * (1 - phi[i])).setName((string("C21_") + "i" + std::to_string(i)).c_str());
                expr21.end();
            }

            // Constraints (22, 23): The number of stages is at least K/2
            for (int k = 1; k <= K / 2; ++k)
            {
                model.add(X[E][k] == 0).setName((string("C22_") + "k" + std::to_string(k)).c_str());
            }
            for (int k = 2; k <= K; ++k)
            {
                model.add(X[S][k] == 0).setName((string("C23_") + "k" + std::to_string(k)).c_str());
            }

            // Constraints (24): No outgoing arc from ending depot
            for (int k = 1; k < K; k++)
            {
                IloExpr c24(env);
                for (int i : V)
                {
                    c24 += X[i][k + 1];
                }
                model.add(1 - X[E][k] >= c24).setName((string("C24_") + "k" + std::to_string(k)).c_str());
                c24.end();
            }

            // Constraint 29: Waiting arcs correspond to drone's sortie
            for (int i : V)
                for (int k = 1; k < K; k++)
                    model.add(Zkk[k][k + 1] >=
                              x[i][i][k])
                        .setName((string("C29_") + "i" + std::to_string(i) + "_k" + std::to_string(k)).c_str());

            // Constraint 30: Stage connectivity
            for (int k = 1; k < K; k++)
            {
                IloExpr lhs_30(env), rhs_30(env);
                for (int i : V)
                {
                    lhs_30 += X[i][k];
                    rhs_30 += X[i][k + 1];
                }
                model.add(lhs_30 >= rhs_30).setName((string("C30_") + "k" + std::to_string(k)).c_str());
                lhs_30.end();
                rhs_30.end();
            }
        }
        // Constraint option 32: No revisit, allow loop
        if (!cfg.allow_revisit)
        {
            for (int i : V)
            {
                IloExpr lhs_32(env);
                for (int k = 1; k <= K - 1; k++)
                    for (int j : V)
                        if (j != i)
                            lhs_32 += x[i][j][k];
                model.add(lhs_32 <= 1).setName((string("C32_") + "i" + std::to_string(i)).c_str());
                lhs_32.end();
            }
        }
        // Constraint option: Allow/not loop
        if (!cfg.allow_loop)
        {
            for (int i : V)
            {
                IloExpr lhs_34(env);
                for (int k = 1; k <= K; k++)
                    lhs_34 += x[i][i][k];
                model.add(lhs_34 == 0).setName((string("C34_") + "i" + std::to_string(i)).c_str());
            }
        }
        // Constraint (36): A launch is considered if truck is available at a given location i of stage k
        for (int i : V)
        {
            for (int k = 1; k <= K; ++k)
            {
                IloExpr launch_expr(env);
                for (int h : C)
                {
                    launch_expr += Y[i][h][k];
                }
                model.add(launch_expr <= X[i][k]).setName((std::string("C36_") + "i" + std::to_string(i) + "_k" + std::to_string(k)).c_str());
                launch_expr.end();
            }
        }
        // Constraint (37): A recovery is considered if truck is available at a given location j of stage k
        for (int j : V)
        {
            for (int k = 1; k <= K; ++k)
            {
                IloExpr recover_expr(env);
                for (int h : C)
                {
                    recover_expr += W[h][j][k];
                }
                model.add(recover_expr <= X[j][k]).setName((std::string("C37_") + "j" + std::to_string(j) + "_k" + std::to_string(k)).c_str());
                recover_expr.end();
            }
        }
        // Constraint (38-39): Drone consistency constraint
        for (int h : C)
        {
            IloExpr drone_launch_expr(env);
            for (int i : V)
            {
                for (int k = 1; k <= K - 1; ++k)
                {
                    drone_launch_expr += Y[i][h][k];
                }
            }
            model.add(drone_launch_expr == phi[h]).setName((std::string("C38_") + "h" + std::to_string(h)).c_str());
            drone_launch_expr.end();
        }
        for (int h : C)
        {
            IloExpr lhs_39(env), rhs_39(env);
            for (int i : V)
                for (int k = 1; k <= K - 1; ++k)
                    lhs_39 += Y[i][h][k];
            for (int j : V)
                for (int k = 2; k <= K; ++k)
                    rhs_39 += W[h][j][k];
            model.add(lhs_39 == rhs_39).setName((std::string("C39_") + "h" + std::to_string(h)).c_str());
            lhs_39.end();
            rhs_39.end();
        }
        // Constrain (40-42): Define a sortie from k to k1
        for (int h : C)
            for (int k = 1; k <= K; k++)
            {
                IloExpr expr_40(env);
                for (int i : V)
                    expr_40 += Y[i][h][k];
                for (int k1 = k + 1; k1 <= K; k1++)
                    expr_40 -= Zhkk[h][k][k1];
                model.add(expr_40 == 0).setName((std::string("C40_") + "h" + std::to_string(h) + "_k" + std::to_string(k)).c_str());
                expr_40.end();
            }
        for (int h : C)
            for (int k1 = 1; k1 <= K; k1++)
            {
                IloExpr expr_41(env);
                for (int j : V)
                    expr_41 += W[h][j][k1];
                for (int k = 1; k <= k1 - 1; k++)
                    expr_41 -= Zhkk[h][k][k1];
                model.add(expr_41 == 0).setName((std::string("C41_") + "h" + std::to_string(h) + "_k1" + std::to_string(k1)).c_str());
                expr_41.end();
            }

        for (int k = 1; k <= K - 1; k++)
            for (int k1 = k + 1; k1 <= K; k1++)
            {
                IloExpr expr_42(env);
                for (int h : C)
                    expr_42 += Zhkk[h][k][k1];
                model.add(Zkk[k][k1] == expr_42).setName((std::string("C42_") + "k" + std::to_string(k) + "_k1" + std::to_string(k1)).c_str());
                expr_42.end();
            }

        // Constraint (43): Drone duration constraint for each customer h
        for (int h : C)
        {
            IloExpr duration_expr(env);
            for (int k = 1; k <= K; ++k)
            {
                for (int i : V)
                {
                    duration_expr += Y[i][h][k] * tau_prime[i][h];
                }
                for (int j : V)
                {
                    duration_expr += W[h][j][k] * tau_prime[h][j];
                }
            }
            model.add(duration_expr <= (D_d - t_R) * phi[h]).setName((std::string("C43_") + "h" + std::to_string(h)).c_str());
            duration_expr.end();
        }

        // Constraint (44): Time drone consistency
        for (int k = 1; k < K; ++k)
        {
            for (int k1 = k + 1; k1 <= K; ++k1)
            {
                IloExpr rhs_44(env);
                IloNum M = 1000;

                rhs_44 += d[k];
                for (int i : V)
                {
                    for (int h : C)
                    {
                        rhs_44 += Y[i][h][k] * tau_prime[i][h];
                    }
                }
                for (int j : V)
                {
                    for (int h : C)
                    {
                        rhs_44 += W[h][j][k1] * tau_prime[h][j];
                    }
                }

                rhs_44 += Zkk[k][k1] * t_R;
                for (int k_bar = k1 + 1; k_bar <= K; ++k_bar)
                {
                    rhs_44 += Zkk[k1][k_bar] * t_L;
                }

                // Adding the constraint dk1 >= dk + drone travel time + recovery/launch time
                model.add(d[k1] + M * (1 - Zkk[k][k1]) >= rhs_44).setName((std::string("C44_") + "k" + std::to_string(k) + "_k1" + std::to_string(k1)).c_str());
                rhs_44.end();
            }
        }

        // Strengthen part specified for 3-index
        if (cfg.test_mode != 1)
        {
            // Constraint (45-46): For each customer and stage, only launch or recover, not both
            for (int h : C)
                for (int k = 1; k <= K; k++)
                {
                    IloExpr lhs_45(env);
                    for (int i : V)
                        lhs_45 += (Y[i][h][k] + W[h][i][k]);
                    model.add(lhs_45 <=
                              phi[h])
                        .setName((std::string("C45_") + "h" + std::to_string(h) + "_k" + std::to_string(k)).c_str());
                    lhs_45.end();
                }

            for (int h : C)
                for (int k = 1; k <= K; k++)
                {
                    IloExpr lhs_46(env);
                    for (int k1 = 1; k1 < k; k1++)
                        lhs_46 += Zhkk[h][k1][k];
                    for (int k1 = k + 1; k1 <= K; k1++)
                        lhs_46 += Zhkk[h][k][k1];
                    model.add(lhs_46 <=
                              phi[h])
                        .setName((std::string("C46_") + "h" + std::to_string(h) + "_k" + std::to_string(k)).c_str());
                    lhs_46.end();
                }

            // Constraint 47:
            for (int h : C)
                for (int k = 1; k <= K - 1; k++)
                {
                    IloExpr lhs_47(env), rhs_47(env);
                    for (int i : V)
                        for (int p = 1; p <= k; p++)
                            lhs_47 += Y[i][h][p];
                    for (int j : V)
                        for (int p = 1; p <= k + 1; p++)
                            rhs_47 += W[h][j][p];
                    model.add(lhs_47 >=
                              rhs_47)
                        .setName((std::string("C47_") + "h" + std::to_string(h) + "_k" + std::to_string(k)).c_str());
                    lhs_47.end();
                    rhs_47.end();
                }
        }

        // Constraint 48:  Jetsuite at specific location
        for (int i : V)
        {
            for (int k = 1; k <= K; ++k)
            {
                IloExpr sum_U(env);
                for (int h : C)
                {
                    sum_U += U[i][h][k];
                }
                model.add(sum_U <= x[i][i][k])
                    .setName((std::string("C48_") + "i" + std::to_string(i) + "_k" + std::to_string(k)).c_str());
                sum_U.end();
            }
        }

        // Contraint 49: Jetsuite consistency
        for (int h : C)
        {
            IloExpr jetsuit_launch_expr(env);
            for (int i : V)
            {
                for (int k = 1; k <= K - 1; ++k)
                {
                    jetsuit_launch_expr += U[i][h][k];
                }
            }
            model.add(jetsuit_launch_expr == phiJetsuite[h]).setName((std::string("C49_") + "h" + std::to_string(h)).c_str());
            jetsuit_launch_expr.end();
        }

        // Contraint 50: Jetsuite duration constraint for each customer h
        for (int h : C)
        {
            IloExpr jetsuit_duration_expr(env);
            for (int k = 1; k <= K - 1; ++k)
            {
                for (int i : V)
                {
                    jetsuit_duration_expr += U[i][h][k] * (tau_prime_prime[i][h] + tau_prime_prime[h][i]);
                }
            }
            model.add(jetsuit_duration_expr <= (D_d - tJ_R) * phiJetsuite[h]).setName((std::string("C50_") + "h" + std::to_string(h)).c_str());
            jetsuit_duration_expr.end();
        }

        // Contraint 51:  Time jetsuite consistency 2 (edited)
        // for (int k = 1; k < K; ++k)
        // {
        //     IloExpr rhs_51(env);
        //     IloExpr lhs_51(env);
        //     IloExpr sum_U(env);
        //     IloNum M = 1000;
        //     rhs_51 += d[k];
        //     lhs_51 += d[k + 1];
        //     for (int i : V)
        //     {
        //         for (int h : C)
        //         {
        //             rhs_51 += U[i][h][k] * (tau_prime_prime[i][h] + tau_prime_prime[h][i]);
        //             rhs_51 += U[i][h][k + 1] * tJ_L;
        //         }
        //     }

        //     for (int i : V)
        //     {
        //         for (int h : C)
        //         {
        //             sum_U += U[i][h][k];
        //         }
        //     }
        //     model.add(lhs_51 + M * (1 - sum_U) >= rhs_51).setName((std::string("C51_") + "k" + std::to_string(k)).c_str());
        //     lhs_51.end();
        //     rhs_51.end();
        // }

        for (int k = 1; k < K; ++k)
        {
            IloExpr rhs_51(env);
            IloExpr lhs_51(env);
            IloExpr sum_U(env);
            IloNum M = 1000;
            rhs_51 += d[k];
            lhs_51 += d[k + 1];

            for (int i : V)
            {
                for (int h : C)
                {
                    rhs_51 += U[i][h][k] * (tau_prime_prime[i][h] + tau_prime_prime[h][i]);
                }
            }

            for (int i : V)
            {
                for (int h : C)
                {
                    sum_U += U[i][h][k];
                }
            }

            // Thêm phần riêng cho k+1
            IloExpr additional_term(env);
            for (int i : V)
            {
                for (int h : C)
                {
                    additional_term += U[i][h][k + 1] * tJ_L;
                }
            }
            rhs_51 += additional_term;

            model.add(lhs_51 + M * (1 - sum_U) >= rhs_51)
                .setName((std::string("C51_") + "k" + std::to_string(k)).c_str());

            lhs_51.end();
            rhs_51.end();
            additional_term.end();
        }

        if (cfg.test_mode == 2)
        {
            // z_{kk'} = 0, k'-k >= 3
            for (int k = 1; k <= K - 3; k++)
                for (int k1 = k + 3; k1 <= K; k1++)
                    model.add(Zkk[k][k1] == 0);
        }

        // Create the CPLEX solver and solve the model
        IloCplex cplex(model);
        // export model
        //        cplex.exportModel("model2.lp");
        //        exit(0);

        // Set the config parameters
        if (cfg.screen_mode == 0 || cfg.screen_mode == 1)
        {
            cplex.setOut(env.getNullStream());
            cplex.setError(env.getNullStream());
            cplex.setWarning(env.getNullStream());
        }
        if (cfg.tl > 0)
            cplex.setParam(IloCplex::Param::TimeLimit, cfg.tl);

        if (cfg.num_thread > 0)
            cplex.setParam(IloCplex::Param::Threads, cfg.num_thread);

        auto startTime = std::chrono::high_resolution_clock::now();
        cplex.solve();
        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

        // Variables to support solution check/write
        vector<int> truck_order;
        vector<vector<int>> drone_order;
        vector<vector<int>> jetsuite_order;
        // Output the results

        if (cfg.screen_mode >= 1)
        {
            cout << "Solve time: " << duration.count() / 1000.0 << std::endl;
            cout << "Objective Value: " << cplex.getObjValue() << std::endl;
            cout << "\nTruck Schedule: " << std::endl;
        }
        for (int k = 1; k <= K; ++k)
        {
            for (int i : V)
            {
                if (cplex.isExtracted(X[i][k]))
                    if (cplex.getValue(X[i][k]) > 0.5)
                    {
                        if (cfg.screen_mode >= 1)
                        {
                            cout << "Stage " << k << ": Truck arrived node " << i << " at " << cplex.getValue(a[k]) << std::endl;

                            for (int j : V)
                                if (cplex.isExtracted(x[i][j][k]))
                                    if (cplex.getValue(x[i][j][k]) > 0.5)
                                        cout << "\tTruck departed from node " << i << " to node " << j
                                             << " at " << cplex.getValue(d[k]) << std::endl;
                        }
                        truck_order.push_back(i);
                    }
            }
        }

        if (cfg.screen_mode >= 1)
        {
            cout << "\nTruck arc:\n";
            for (int k = 1; k <= K; ++k)
                for (int i : V)
                    for (int j : V)
                        if (cplex.isExtracted(x[i][j][k]))
                            if (cplex.getValue(x[i][j][k]) > 0.5)
                                cout << "Stage " << k << ": " << i << " -> " << j << std::endl;
        }

        if (cfg.screen_mode >= 1)
        {
            cout << "\nDrone Schedule: " << std::endl;
        }
        for (int k = 1; k <= K; ++k)
        {
            for (int k1 = 1; k1 <= K; ++k1)
            {
                for (int i : V)
                {
                    for (int j : V)
                    {
                        for (int h : C)
                        {
                            // Check if the drone was launched from i to serve h at stage k and recovered at j at stage k1
                            if (cplex.isExtracted(Y[i][h][k]) && cplex.isExtracted(W[h][j][k1]))
                            {
                                if (cplex.getValue(Y[i][h][k]) > 0.5 && cplex.getValue(W[h][j][k1]) > 0.5)
                                {
                                    if (cfg.screen_mode >= 1)
                                    {
                                        cout << "Stage " << k << " to " << k1 << ": Drone departed from node " << i << " visited customer " << h << " and arrived to node " << j << std::endl;
                                        cout << "\t Departed at: " << cplex.getValue(d[k]) << "\n";
                                        cout << "\t Visited at: " << cplex.getValue(d[k] + tau_prime[i][h]) << "\n";
                                        cout << "\t Arrived at: "
                                             << cplex.getValue(d[k] + tau_prime[i][h] + tau_prime[h][j]) << "\n";
                                    }

                                    // Kiểm tra giá trị k, k1 và truck_order trước khi thêm vào drone_order
                                    if (k >= 1 && k1 >= 1 && k - 1 >= 0 && k1 - 1 >= 0)
                                    {
                                        // Kiểm tra xem k-1 và k1-1 có tương ứng với vị trí hợp lệ trong truck_order
                                        int launch_pos = k - 1;
                                        int recovery_pos = k1 - 1;

                                        if (launch_pos < (int)truck_order.size() && recovery_pos < (int)truck_order.size())
                                        {
                                            // -1 because solution class receives 0-indexed
                                            drone_order.push_back({k - 1, h, k1 - 1});
                                            cout << "Added drone operation: position " << k - 1 << " (node " << i
                                                 << ") -> customer " << h << " -> position " << k1 - 1
                                                 << " (node " << j << ")" << std::endl;
                                        }
                                        else
                                        {
                                            cerr << "Warning: Invalid launch/recovery position for drone trip: "
                                                 << launch_pos << "/" << recovery_pos
                                                 << " (truck_order size: " << truck_order.size() << ")" << std::endl;
                                        }
                                    }
                                    else
                                    {
                                        cerr << "Warning: Invalid stage values for drone: k=" << k << ", k1=" << k1 << std::endl;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // print out Y[i][h][k] and W[h][j][k] to test 3-index formulation only. The printing above is enough
        if (cfg.screen_mode >= 3)
        {
            cout << "\nDrone launch:\n";
            for (int k = 1; k <= K; k++)
                for (int i : V)
                    for (int h : C)
                        if (cplex.isExtracted(Y[i][h][k]))
                            if (cplex.getValue(Y[i][h][k]) > 0.5)
                                cout << "Stage " << k << ": " << i << " -> " << h << std::endl;

            cout << "\nDrone recover:\n";
            for (int k = 1; k <= K; k++)
                for (int h : C)
                    for (int j : V)
                        if (cplex.isExtracted(W[h][j][k]))
                            if (cplex.getValue(W[h][j][k]) > 0.5)
                                cout << "Stage " << k << ": " << h << " -> " << j << std::endl;
        }

        // Jetsuite Schedule
        if (cfg.screen_mode >= 1)
        {
            cout << "\nJetsuite Schedule: " << std::endl;
        }

        for (int k = 2; k <= K; ++k) // typically jetsuite operations do not occur at stage 1 (depot)
        {
            for (int i : V)
            {
                for (int h : C)
                {
                    if (cplex.isExtracted(U[i][h][k]) && cplex.getValue(U[i][h][k]) > 0.5)
                    {
                        double launch_time = cplex.getValue(d[k]);                           // Jetsuite launch time
                        double visit_time = launch_time + instance->tau_prime_prime[i][h];   // Time at customer
                        double recovery_time = visit_time + instance->tau_prime_prime[h][i]; // Time returning to launch node

                        cout << "Stage " << k << ": Jetsuite launched from node " << i
                             << " served customer " << h << " and recovered at node " << i << std::endl;
                    }
                }
            }
        }

        for (int i : V)
        {
            for (int j : V)
            {
                for (int k = 1; k <= K; ++k)
                {
                    if (cplex.isExtracted(x[i][j][k]) && cplex.getValue(x[i][j][k]) > 0.5)
                    {

                        cout << "x[" << i << "][" << j << "][" << k << "] = " << cplex.getValue(x[i][j][k]) << std::endl;
                    }
                }
            }
        }

        cout << "\nDrone Variables (V[i][h][k]):" << std::endl;
        for (int i : V)
        {
            for (int h : C)
            {
                for (int k = 1; k <= K; ++k)
                {
                    if (cplex.isExtracted(Y[i][h][k]) && cplex.getValue(Y[i][h][k]) > 0.5)
                    {
                        cout << "Y[" << i << "][" << h << "][" << k << "] = "
                             << cplex.getValue(Y[i][h][k]) << std::endl;
                    }
                }
            }
        }

        cout << "\nJetsuite Variables (U[i][h][k]):" << std::endl;
        for (int i : V)
        {
            for (int h : C)
            {
                for (int k = 1; k <= K; ++k)
                {
                    if (cplex.isExtracted(U[i][h][k]) && cplex.getValue(U[i][h][k]) > 0.5)
                    {
                        cout << "U[" << i << "][" << h << "][" << k << "] = "
                             << cplex.getValue(U[i][h][k]) << std::endl;
                    }
                }
            }
        }

        // Additional output for debugging if screen_mode >= 3
        if (cfg.screen_mode >= 3)
        {
            cout << "\nJetsuite launch/recovery details:\n";
            for (int k = 1; k <= K; ++k)
            {
                for (int i : V)
                {
                    for (int h : C)
                    {
                        if (cplex.isExtracted(U[i][h][k]) && cplex.getValue(U[i][h][k]) > 0.5)
                        {
                            cout << "Stage " << k << ": Jetsuite launched from node " << i << " to serve customer " << h
                                 << std::endl;
                        }
                    }
                }
            }
        }

        double c = cplex.getObjValue();
        double lowerBound = cplex.getBestObjValue();
        double gap = round(cplex.getMIPRelativeGap() * 10000.0) / 100.0;
        std::sort(drone_order.begin(), drone_order.end(), [](const std::vector<int> &a, const std::vector<int> &b)
                  { return a[0] < b[0]; });
        double solve_time = duration.count() / 1000.0;
        Solution solution(c, lowerBound, gap, solve_time, instance, &cfg, truck_order, drone_order, jetsuite_order);
        solution.write();
        if (cfg.is_check)
            if (!solution.isFeasible())
                cerr << "WARNING: The instance " << instance->folder_path << " solves infeasible!";
        return Result{c, duration.count() / 1000.0, gap};
    }
    catch (IloException &e)
    {
        std::cerr << "Concert exception caught: " << e << std::endl;
    }
    catch (...)
    {
        std::cerr << "Unknown exception caught" << std::endl;
    }

    return Result{-1, -1, -1};
}

Result FSTSPSolver::FSTSP_2index_with_JetSuite(Config &cfg) const
{
    IloEnv env;
    try
    {
        // ===== Parameters (NO STAGE) =====
        int N = instance->num_node - 1; // customers = 1..N
        int S = 0;                      // depot (node 0)

        // Endurance / service times
        double E_d = cfg.dtl; // drone endurance
        double t_L = cfg.sl;  // drone launch
        double t_R = cfg.sr;  // drone recovery
        double E_J = cfg.dtl; // jetsuite endurance (nếu có trường riêng, thay ở đây)
        double tJ_L = cfg.sl; // jetsuite launch
        double tJ_R = cfg.sr; // jetsuite recovery

        // Big-M (sẽ tinh chỉnh theo data bound)
        double M = 1e6;

        // ===== Sets =====
        std::set<int> C, V;
        for (int i = 1; i <= N; ++i)
            C.insert(i);
        V = C;
        V.insert(S); // V = {0} ∪ C

        // ===== Travel-time matrices =====
        std::vector<std::vector<double>> tau = instance->tau;                         // τ_ij (truck)
        std::vector<std::vector<double>> tau_prime = instance->tau_prime;             // τ'_ij (drone)
        std::vector<std::vector<double>> tau_prime_prime = instance->tau_prime_prime; // τ''_ij (jetsuite)

        // ===== Model =====
        IloModel model(env);

        // ==============================
        // Decision Variables (2-index, no-stage, style cũ)
        // ==============================

        // x_{ij} ∈ {0,1}  (truck arc i->j), i,j ∈ V
        IloArray<IloBoolVarArray> x(env, N + 1);
        for (int i : V)
        {
            x[i] = IloBoolVarArray(env, N + 1);
            for (int j : V)
            {
                std::stringstream nm;
                nm << "x_" << i << "_" << j;
                x[i][j] = IloBoolVar(env, 0, 1, nm.str().c_str());
            }
        }

        // y^D_{ij} ∈ {0,1}  (drone launch at i, serve j, return i), i ∈ V, j ∈ C
        IloArray<IloBoolVarArray> yD(env, N + 1);
        for (int i : V)
        {
            yD[i] = IloBoolVarArray(env, N + 1);
            for (int j : C)
            {
                std::stringstream nm;
                nm << "yD_" << i << "_" << j;
                yD[i][j] = IloBoolVar(env, 0, 1, nm.str().c_str());
            }
        }

        // y^J_{ik} ∈ {0,1}  (jetsuite launch at i, serve k, return i), i ∈ V, k ∈ C
        IloArray<IloBoolVarArray> yJ(env, N + 1);
        for (int i : V)
        {
            yJ[i] = IloBoolVarArray(env, N + 1);
            for (int k : C)
            {
                std::stringstream nm;
                nm << "yJ_" << i << "_" << k;
                yJ[i][k] = IloBoolVar(env, 0, 1, nm.str().c_str());
            }
        }

        // u_i ∈ [1, |C|] cho i∈C; u_0 = 0 (MTZ)
        IloNumVarArray u(env, N + 1, 0, N, ILOINT); // dùng ILOINT cho MTZ chuẩn
        for (int i : V)
        {
            std::stringstream nm;
            nm << "u_" << i;
            u[i].setName(nm.str().c_str());
        }
        // domain/detail cho u: depot = 0, khách hàng trong [1..|C|]
        model.add(u[0] == 0).setName("u_depot_fixed");
        for (int i : C)
        {
            model.add(u[i] >= 1).setName((std::string("u_lower_") + std::to_string(i)).c_str());
            model.add(u[i] <= (int)C.size()).setName((std::string("u_upper_") + std::to_string(i)).c_str());
        }

        // T_i ≥ 0  (truck waiting time), i ∈ V
        IloNumVarArray T(env, N + 1, 0, IloInfinity, ILOFLOAT);
        for (int i : V)
        {
            std::stringstream nm;
            nm << "T_" << i;
            T[i].setName(nm.str().c_str());
        }

        // a_i ≥ 0  (truck arrival time), i ∈ V
        IloNumVarArray a(env, N + 1, 0, IloInfinity, ILOFLOAT);
        for (int i : V)
        {
            std::stringstream nm;
            nm << "a_" << i;
            a[i].setName(nm.str().c_str());
        }

        // z_{ikj} ∈ {0,1} (Jetsuite–Drone coordination), i ∈ V, k ∈ C, j ∈ V
        IloArray<IloArray<IloBoolVarArray>> z(env, N + 1);
        for (int i : V)
        {
            z[i] = IloArray<IloBoolVarArray>(env, N + 1);
            for (int k : C)
            {
                z[i][k] = IloBoolVarArray(env, N + 1);
                for (int j : V)
                {
                    std::stringstream nm;
                    nm << "z_" << i << "_" << k << "_" << j;
                    z[i][k][j] = IloBoolVar(env, 0, 1, nm.str().c_str());
                }
            }
        }

        // Cấm loop truck: x_{ii} = 0 (đặt ngay sau khi khai báo x cho rõ ràng)
        for (int i : V)
        {
            model.add(x[i][i] == 0).setName((std::string("no_loop_") + std::to_string(i)).c_str());
        }

        // ===== Objective (eq:objective_2index) =====
        {
            IloExpr obj(env);

            // Σ τ_ij x_ij
            for (int i : V)
                for (int j : V)
                    obj += tau[i][j] * x[i][j];

            // Σ (2 τ'_{ij} + t_L + t_R) y^D_{ij}, j ∈ C
            for (int i : V)
                for (int j : C)
                    obj += (2.0 * tau_prime[i][j] + t_L + t_R) * yD[i][j];

            // Σ (2 τ''_{ik} + t_JL + t_JR) y^J_{ik}, k ∈ C
            for (int i : V)
                for (int k : C)
                    obj += (2.0 * tau_prime_prime[i][k] + tJ_L + tJ_R) * yJ[i][k];

            // + Σ T_i
            for (int i : V)
                obj += T[i];

            model.add(IloMinimize(env, obj));
            obj.end();
        }

        // ===== Truck Route Constraints =====

        // (truck_start)  sum_{j∈C} x_{0j} = 1
        {
            IloExpr expr(env);
            for (int j : C)
                expr += x[S][j];
            model.add(expr == 1).setName("truck_start");
            expr.end();
        }

        // (truck_end)  sum_{i∈C} x_{i0} = 1
        {
            IloExpr expr(env);
            for (int i : C)
                expr += x[i][S];
            model.add(expr == 1).setName("truck_end");
            expr.end();
        }

        // (no loop) x_{ii} = 0
        for (int i : V)
        {
            model.add(x[i][i] == 0).setName((std::string("no_loop_") + std::to_string(i)).c_str());
        }

        // (truck_out_degree)  Σ_j x_{ij} ≤ 1, ∀ i∈C
        for (int i : C)
        {
            IloExpr expr(env);
            for (int j : V)
                expr += x[i][j];
            model.add(expr <= 1).setName((std::string("truck_out_degree_") + std::to_string(i)).c_str());
            expr.end();
        }

        // (truck_in_degree)  Σ_i x_{ji} ≤ 1, ∀ j∈C
        for (int j : C)
        {
            IloExpr expr(env);
            for (int i : V)
                expr += x[i][j];
            model.add(expr <= 1).setName((std::string("truck_in_degree_") + std::to_string(j)).c_str());
            expr.end();
        }

        // (a_0)  a_0 = 0
        model.add(a[S] == 0).setName("a_0");
        // (truck_propagation) apply only when j is a customer (avoid j = 0)

        for (int i : V)
            for (int j : C)
                if (i != j)
                {
                    IloExpr expr(env);
                    expr = a[i] + T[i] + tau[i][j] - M * (1 - x[i][j]);
                    model.add(a[j] >= expr)
                        .setName((std::string("truck_propagation_") + std::to_string(i) + "_" + std::to_string(j)).c_str());
                    expr.end();
                }

        // (mtz_subtour)  u_i − u_j + |C| x_ij ≤ |C| − 1, ∀ i≠j, i,j∈C
        int nC = (int)C.size();
        for (int i : C)
            for (int j : C)
                if (i != j)
                {
                    IloExpr expr(env);
                    expr = u[i] - u[j] + nC * x[i][j];
                    model.add(expr <= nC - 1).setName((std::string("mtz_") + std::to_string(i) + "_" + std::to_string(j)).c_str());
                    expr.end();
                }
        // fix depot in MTZ (thường đặt u_0 = 0)
        model.add(u[S] == 0).setName("mtz_u0");

        // (truck flow)  Σ_j x_{ij} = Σ_j x_{ji}, ∀ i∈C
        for (int i : C)
        {
            IloExpr out(env), in(env);
            for (int j : V)
                out += x[i][j];
            for (int j : V)
                in += x[j][i];
            model.add(out == in).setName((std::string("truck_flow_") + std::to_string(i)).c_str());
            out.end();
            in.end();
        }

        // ===== Drone Constraints =====

        // (drone_endurance)  Σ_{i∈V} y^D_{ij} (2 τ'_{ij} + t_R) ≤ E_d, ∀ j∈C
        for (int j : C)
        {
            IloExpr expr(env);
            for (int i : V)
                expr += yD[i][j] * (2.0 * tau_prime[i][j] + t_R);
            model.add(expr <= E_d).setName((std::string("drone_endurance_") + std::to_string(j)).c_str());
            expr.end();
        }

        // (at_most_1_drone)  Σ_{j∈C} y^D_{ij} ≤ 1, ∀ i∈V
        for (int i : V)
        {
            IloExpr expr(env);
            for (int j : C)
                expr += yD[i][j];
            model.add(expr <= 1).setName((std::string("drone_atmost1_") + std::to_string(i)).c_str());
            expr.end();
        }

        // (drone_truck_presence)  y^D_{ij} ≤ Σ_{v∈V} x_{iv}, ∀ i∈V, j∈C
        for (int i : V)
        {
            IloExpr deg_out(env);
            for (int v : V)
                deg_out += x[i][v];
            for (int j : C)
            {
                model.add(yD[i][j] <= deg_out).setName((std::string("drone_truck_presence_") + std::to_string(i) + "_" + std::to_string(j)).c_str());
            }
            deg_out.end();
        }

        // (drone_after_depot)  Σ_{j∈C} y^D_{0j} = 0
        {
            IloExpr expr(env);
            for (int j : C)
                expr += yD[S][j];
            model.add(expr == 0).setName("drone_after_depot");
            expr.end();
        }

        // (truck_waiting_time_drone)  T_i ≥ Σ_{j∈C} (2 τ'_{ij} + t_L + t_R) y^D_{ij}, ∀ i∈V
        for (int i : V)
        {
            IloExpr expr(env);
            for (int j : C)
                expr += (2.0 * tau_prime[i][j] + t_L + t_R) * yD[i][j];
            model.add(T[i] >= expr).setName((std::string("wait_drone_") + std::to_string(i)).c_str());
            expr.end();
        }

        // ===== Jetsuite Constraints =====

        // (jetsuite endurance)  Σ_{i∈V} y^J_{ik} (2 τ''_{ik} + t_JR) ≤ E_J, ∀ k∈C
        for (int k : C)
        {
            IloExpr expr(env);
            for (int i : V)
                expr += yJ[i][k] * (2.0 * tau_prime_prime[i][k] + tJ_R);
            model.add(expr <= E_J).setName((std::string("jetsuite_endurance_") + std::to_string(k)).c_str());
            expr.end();
        }

        // (coupling_constraints)  z_{ikj} ≤ y^J_{ik}, z_{ikj} ≤ y^D_{ij}, y^D_{ij} + y^J_{ik} − z_{ikj} ≤ 1
        for (int i : V)
            for (int k : C)
                for (int j : V)
                {
                    model.add(z[i][k][j] <= yJ[i][k]).setName((std::string("z_le_yJ_") + std::to_string(i) + "_" + std::to_string(k) + "_" + std::to_string(j)).c_str());
                    model.add(z[i][k][j] <= yD[i][j]).setName((std::string("z_le_yD_") + std::to_string(i) + "_" + std::to_string(k) + "_" + std::to_string(j)).c_str());
                    IloExpr expr(env);
                    expr = yD[i][j] + yJ[i][k] - z[i][k][j];
                    model.add(expr <= 1).setName(
                        (std::string("yD_plus_yJ_minus_z_le1_") + std::to_string(i) + "_" + std::to_string(k) + "_" + std::to_string(j)).c_str());
                    expr.end();
                }

        // (joint_endurance_constraint)  Σ_{i∈V} z_{ikj} (2 τ''_{ik} + t_JR) ≤ min(E_J, E_d), ∀ k∈C, ∀ j∈V
        // (công thức gốc có "i≠k" trong phần mô tả; nếu cần áp dụng, lọc thêm điều kiện i!=k)
        double EJd = std::min(E_J, E_d);
        for (int k : C)
            for (int j : V)
            {
                IloExpr expr(env);
                for (int i : V)
                {
                    expr += z[i][k][j] * (2.0 * tau_prime_prime[i][k] + tJ_R);
                }
                model.add(expr <= EJd).setName((std::string("joint_endurance_k_") + std::to_string(k) + "_j_" + std::to_string(j)).c_str());
                expr.end();
            }

        // (at_most_1_jetsuite)  Σ_{k∈C} y^J_{ik} ≤ 1, ∀ i∈V
        for (int i : V)
        {
            IloExpr expr(env);
            for (int k : C)
                expr += yJ[i][k];
            model.add(expr <= 1).setName((std::string("jetsuite_atmost1_") + std::to_string(i)).c_str());
            expr.end();
        }

        // (jetsuite_truck_presence)  y^J_{ik} ≤ Σ_{j∈V} x_{ij}, ∀ i∈V,k∈C
        for (int i : V)
        {
            IloExpr deg_out(env);
            for (int j : V)
                deg_out += x[i][j];
            for (int k : C)
            {
                model.add(yJ[i][k] <= deg_out).setName((std::string("jetsuite_truck_presence_") + std::to_string(i) + "_" + std::to_string(k)).c_str());
            }
            deg_out.end();
        }

        // (jetsuite_after_depot)  Σ_{k∈C} y^J_{0k} = 0
        {
            IloExpr expr(env);
            for (int k : C)
                expr += yJ[S][k];
            model.add(expr == 0).setName("jetsuite_after_depot");
            expr.end();
        }

        // Each customer is served by exactly one mode
        for (int j : C)
        {
            IloExpr served(env);
            for (int i : V)
                served += x[i][j]; // truck (incoming to j)
            for (int i : V)
                served += yD[i][j]; // drone
            for (int i : V)
                served += yJ[i][j]; // jetsuite
            model.add(served == 1).setName((std::string("customer_served_once_") + std::to_string(j)).c_str());
            served.end();
        }

        // (truck_waiting_time_jetsuite)  T_i ≥ Σ_{k∈C} (2 τ''_{ik} + t_JL + t_JR) y^J_{ik}, ∀ i∈V
        for (int i : V)
        {
            IloExpr expr(env);
            for (int k : C)
                expr += (2.0 * tau_prime_prime[i][k] + tJ_L + tJ_R) * yJ[i][k];
            model.add(T[i] >= expr).setName((std::string("wait_jetsuite_") + std::to_string(i)).c_str());
            expr.end();
        }

        // Create the CPLEX solver and solve the model
        IloCplex cplex(model);

        // Set the config parameters
        if (cfg.screen_mode == 0 || cfg.screen_mode == 1)
        {
            cplex.setOut(env.getNullStream());
            cplex.setError(env.getNullStream());
            cplex.setWarning(env.getNullStream());
        }
        if (cfg.tl > 0)
            cplex.setParam(IloCplex::Param::TimeLimit, cfg.tl);
        if (cfg.num_thread > 0)
            cplex.setParam(IloCplex::Param::Threads, cfg.num_thread);

        auto startTime = std::chrono::high_resolution_clock::now();
        cplex.solve();
        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

        /// ==== Solve time already measured above ====

        if (cfg.screen_mode >= 1)
        {
            cout << "Solve time: " << duration.count() / 1000.0 << std::endl;
            cout << "Objective Value: " << cplex.getObjValue() << std::endl;
            cout << "\nTruck Schedule (no-stage): " << std::endl;
        }

        // ==============================
        // Reconstruct truck route from 2-index x_{ij}
        // ==============================
        vector<int> truck_order;               // order of nodes (start at 0)
        std::unordered_map<int, int> node_pos; // node -> position on route
        vector<int> next_of(N + 1, -1);
        vector<int> indeg(N + 1, 0), outdeg(N + 1, 0);

        // collect arcs with value 1
        for (int i : V)
            for (int j : V)
            {
                if (i == j)
                    continue;
                if (cplex.isExtracted(x[i][j]) && cplex.getValue(x[i][j]) > 0.5)
                {
                    next_of[i] = j;
                    outdeg[i]++;
                    indeg[j]++;
                    if (cfg.screen_mode >= 1)
                        cout << "Truck arc: " << i << " -> " << j << "\n";
                }
            }

        // traverse from depot 0
        {
            int cur = 0;
            std::unordered_set<int> seen; // chỉ còn biến này trong block
            truck_order.push_back(cur);
            node_pos[cur] = 0; // dùng node_pos ở ngoài

            for (int step = 0; step <= N + 5; ++step)
            {
                if (next_of[cur] == -1)
                    break;
                int nxt = next_of[cur];

                // nếu quay về depot sau ít nhất 1 bước -> đóng tuyến
                if (nxt == 0 && step > 0)
                {
                    truck_order.push_back(0);
                    node_pos[0] = (int)truck_order.size() - 1;
                    break;
                }
                // chặn cycle không quay về 0
                if (seen.count(nxt))
                    break;
                seen.insert(nxt);

                truck_order.push_back(nxt);
                node_pos[nxt] = (int)truck_order.size() - 1;
                cur = nxt;
            }
        }

        // nếu vì lý do nào đó chưa khép về 0, ép khép
        if (truck_order.empty() || truck_order.back() != 0)
        {
            truck_order.push_back(0);
            node_pos[0] = (int)truck_order.size() - 1;
        }

        // ==============================
        // Drone sorties (launch i, serve j, return i)
        // ==============================
        vector<vector<int>> drone_order; // {launch_pos, customer, recover_pos}
        if (cfg.screen_mode >= 1)
            cout << "\nDrone Schedule (no-stage):\n";

        for (int i : V)
        {
            for (int j : C)
            {
                if (cplex.isExtracted(yD[i][j]) && cplex.getValue(yD[i][j]) > 0.5)
                {
                    double ai = cplex.isExtracted(a[i]) ? cplex.getValue(a[i]) : 0.0;
                    double depart = ai;
                    double visit = depart + t_L + instance->tau_prime[i][j];
                    double retTime = depart + (2.0 * instance->tau_prime[i][j]) + t_L + t_R;

                    if (cfg.screen_mode >= 1)
                    {
                        cout << "Drone: launch " << i << " -> serve " << j << " -> return " << i
                             << " | depart≈" << depart
                             << ", visit≈" << visit
                             << ", return≈" << retTime << "\n";
                    }

                    auto it = node_pos.find(i);
                    if (it != node_pos.end())
                    {
                        int pos = it->second;
                        if (pos < (int)truck_order.size() - 1)
                        {
                            int recover_pos = pos + 1; // bắt buộc > pos
                            drone_order.push_back({pos, j, recover_pos});
                        }
                        else if (cfg.screen_mode >= 2)
                        {
                            std::cerr << "[skip drone] launch at last route position (node "
                                      << i << ") => no next step for recovery\n";
                        }
                    }
                }
            }
        }

        // ==============================
        // Jetsuite sorties (launch i, serve k, return i)
        // ==============================
        vector<vector<int>> jetsuite_order; // {launch_pos, customer, recover_pos}
        if (cfg.screen_mode >= 1)
            cout << "\nJetsuite Schedule (no-stage):\n";

        for (int i : V)
        {
            for (int k : C)
            {
                if (cplex.isExtracted(yJ[i][k]) && cplex.getValue(yJ[i][k]) > 0.5)
                {
                    double ai = cplex.isExtracted(a[i]) ? cplex.getValue(a[i]) : 0.0;
                    double depart = ai;
                    double visit = depart + tJ_L + instance->tau_prime_prime[i][k];
                    double retTime = depart + (2.0 * instance->tau_prime_prime[i][k]) + tJ_L + tJ_R;

                    if (cfg.screen_mode >= 1)
                    {
                        cout << "Jetsuite: launch " << i << " -> serve " << k << " -> return " << i
                             << " | depart≈" << depart
                             << ", visit≈" << visit
                             << ", return≈" << retTime << "\n";
                    }

                    auto it = node_pos.find(i);
                    if (it != node_pos.end())
                    {
                        int pos = it->second;
                        if (pos < (int)truck_order.size() - 1)
                        {
                            int recover_pos = pos + 1; // bắt buộc > pos
                            jetsuite_order.push_back({pos, k, recover_pos});
                        }
                        else if (cfg.screen_mode >= 2)
                        {
                            std::cerr << "[skip jetsuite] launch at last route position (node "
                                      << i << ") => no next step for recovery\n";
                        }
                    }
                }
            }
        }

        // ==============================
        // Summaries
        // ==============================
        if (cfg.screen_mode >= 1)
        {
            cout << "\nTruck route (order): ";
            for (size_t t = 0; t < truck_order.size(); ++t)
            {
                if (t)
                    cout << " -> ";
                cout << truck_order[t];
            }
            cout << "\n";
        }

        // ==============================
        // Prepare Solution (reuse existing class)
        // - It used to expect stage-based positions; now we pass route positions (0-index).
        // ==============================
        double c = cplex.getObjValue();
        double lb = cplex.getBestObjValue();
        double gap = round(cplex.getMIPRelativeGap() * 10000.0) / 100.0;

        // If your Solution::write() assumes certain invariants, keep this structure:
        //   truck_order: order of nodes
        //   drone_order: list of {launch_pos_on_route, customer, recover_pos_on_route}
        //   jetsuite_order: same scheme as above
        Solution solution(c, lb, gap, duration.count() / 1000.0, instance, &cfg,
                          truck_order, drone_order, jetsuite_order);
        solution.write();
        if (cfg.is_check && !solution.isFeasible())
        {
            cerr << "WARNING: The instance " << instance->folder_path << " solves infeasible!";
        }

        return Result{c, duration.count() / 1000.0, gap};
    }
    catch (IloException &e)
    {
        std::cerr << "Concert exception: " << e << std::endl;
    }
    catch (...)
    {
        std::cerr << "Unknown exception" << std::endl;
    }
    return Result{-1, -1, -1};
}

Result FSTSPSolver::FSTSP_3indexNoStage_with_JetSuite(Config &cfg) const
{
    IloEnv env;
    try
    {
        int N = instance->num_node - 1; // đổi thành -2 nếu dữ liệu có depot về riêng
        int S = 0;
        std::set<int> C;
        for (int i = 1; i <= N; ++i)
            C.insert(i);
        std::set<int> V = C;
        V.insert(S);

        auto &tau = instance->tau;
        auto &tauD = instance->tau_prime;
        auto &tauJ = instance->tau_prime_prime;

        double E_d = cfg.dtl;
        double tL = cfg.sl, tR = cfg.sr;
        double E_J = cfg.ej;
        double tJL = cfg.tJ_L, tJR = cfg.tJ_R;
        double M = cfg.bigM;

        IloModel model(env);

        // Truck arcs
        IloArray<IloBoolVarArray> x(env, N + 1);
        for (int i : V)
        {
            x[i] = IloBoolVarArray(env, N + 1);
            for (int j : V)
            {
                std::stringstream nm;
                nm << "x_" << i << "_" << j;
                x[i][j] = IloBoolVar(env, nm.str().c_str());
            }
        }
        for (int i : V)
            model.add(x[i][i] == 0).setName((std::string("no_loop_") + std::to_string(i)).c_str());

        // JetSuite yJ_ik
        IloArray<IloBoolVarArray> yJ(env, N + 1);
        for (int i : V)
        {
            yJ[i] = IloBoolVarArray(env, N + 1);
            for (int k : C)
            {
                std::stringstream nm;
                nm << "yJ_" << i << "_" << k;
                yJ[i][k] = IloBoolVar(env, nm.str().c_str());
            }
        }

        // Drone 3-index yD3_iju  [NEW]
        IloArray<IloArray<IloBoolVarArray>> yD3(env, N + 1);
        for (int i : V)
        {
            yD3[i] = IloArray<IloBoolVarArray>(env, N + 1);
            for (int j : C)
            {
                yD3[i][j] = IloBoolVarArray(env, N + 1);
                for (int uNode : V)
                {
                    std::stringstream nm;
                    nm << "yD3_" << i << "_" << j << "_" << uNode;
                    yD3[i][j][uNode] = IloBoolVar(env, nm.str().c_str());
                }
            }
        }
        // Drone waiting time wD_iju  [NEW]
        IloArray<IloArray<IloNumVarArray>> wD(env, N + 1);
        for (int i : V)
        {
            wD[i] = IloArray<IloNumVarArray>(env, N + 1);
            for (int j : C)
            {
                wD[i][j] = IloNumVarArray(env, N + 1);
                for (int uNode : V)
                {
                    std::stringstream nm;
                    nm << "wD_" << i << "_" << j << "_" << uNode;
                    wD[i][j][uNode] = IloNumVar(env, 0.0, IloInfinity, ILOFLOAT);
                    wD[i][j][uNode].setName(nm.str().c_str()); // wD >= 0
                }
            }
        }

        // MTZ and times
        IloNumVarArray u(env, N + 1, 0, N, ILOFLOAT);
        for (int i : C)
        {
            std::stringstream nm;
            nm << "u_" << i;
            u[i].setName(nm.str().c_str());
        }

        IloNumVarArray a(env, N + 1, 0, IloInfinity, ILOFLOAT);
        for (int i : V)
        {
            std::stringstream nm;
            nm << "a_" << i;
            a[i].setName(nm.str().c_str());
        }
        IloNumVarArray T(env, N + 1, 0, IloInfinity, ILOFLOAT);
        for (int i : V)
        {
            std::stringstream nm;
            nm << "T_" << i;
            T[i].setName(nm.str().c_str());
        }
        // departure time d[i]  [NEW]
        IloNumVarArray d(env, N + 1, 0, IloInfinity, ILOFLOAT);
        for (int i : V)
        {
            std::stringstream nm;
            nm << "d_" << i;
            d[i].setName(nm.str().c_str());
        }
        for (int i : V)
            model.add(d[i] >= a[i] + T[i]).setName((std::string("dep_ge_arr_wait_") + std::to_string(i)).c_str());

        // ---- Objective (Mode 31 mimic Mode 3: minimize truck completion time) ----

        // Biến makespan của truck: thời điểm rời node muộn nhất
        IloNumVar T_end(env, 0.0, IloInfinity, ILOFLOAT);
        T_end.setName("T_end");

        // T_end >= d[i] cho mọi node i mà truck ghé
        for (int i : V)
        {
            model.add(T_end >= d[i])
                .setName((std::string("Tend_ge_d_") + std::to_string(i)).c_str());
        }

        // Minimize thời điểm hoàn thành của truck
        model.add(IloMinimize(env, T_end));

        // ---- Truck route ----
        {
            IloExpr e(env);
            for (int j : C)
                e += x[S][j];
            model.add(e == 1).setName("truck_start");
            e.end();
        }
        {
            IloExpr e(env);
            for (int i : C)
                e += x[i][S];
            model.add(e == 1).setName("truck_end");
            e.end();
        }
        for (int i : C)
        {
            IloExpr out(env);
            for (int j : V)
                out += x[i][j];
            IloExpr in(env);
            for (int j : V)
                in += x[j][i];
            model.add(out <= 1).setName((std::string("deg_out_") + std::to_string(i)).c_str());
            model.add(in <= 1).setName((std::string("deg_in_") + std::to_string(i)).c_str());
            model.add(out == in).setName((std::string("flow_") + std::to_string(i)).c_str());
            out.end();
            in.end();
        }
        model.add(a[S] == 0).setName("a_0");

        // Chỉ propagate thời gian tới các customer (j ∈ C), không tới depot
        for (int i : V)
            for (int j : C)
                if (i != j)
                {
                    IloExpr rhs(env);
                    rhs += a[i] + T[i] + tau[i][j] - M * (1 - x[i][j]);
                    model.add(a[j] >= rhs)
                        .setName((std::string("time_") + std::to_string(i) + "_" + std::to_string(j)).c_str());
                    rhs.end();
                }

        for (int i : C)
            for (int j : C)
                if (i != j)
                {
                    model.add(u[i] - u[j] + ((int)C.size()) * x[i][j] <= (int)C.size() - 1)
                        .setName((std::string("mtz_") + std::to_string(i) + "_" + std::to_string(j)).c_str());
                }

        // ---- Drone (3-index) ----
        // // endurance per (i,j,u)
        // for (int i : V) for (int j : C) for (int uNode : V) {
        //     IloExpr rhs(env);
        //     rhs += tauD[i][j] + tauD[j][uNode] + tR;
        //     model.add( rhs <= E_d + M*(1 - yD3[i][j][uNode]) )
        //         .setName(("dr_end_case2_"+std::to_string(i)+"_"+std::to_string(j)+"_"+std::to_string(uNode)).c_str());
        //     rhs.end();
        // }
        // at most 1 launch per i

        // ---- Drone (3-index) ----
        // Waiting time wD_iju và endurance như Case 2 trong PDF
        for (int i : V)
            for (int j : C)
                for (int uNode : V)
                {
                    // (32) wD_iju <= M * yD3_iju  (wD = 0 nếu sortie không được chọn)
                    model.add(wD[i][j][uNode] <= M * yD3[i][j][uNode])
                        .setName(("dr_wait_link_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(uNode)).c_str());

                    // (31) wD_iju >= a[u] - (a[i] + tL + τ'ij + τ'ju) - M(1 - yD3_iju)
                    //     (drone đến u sớm, phải chờ truck)
                    {
                        IloExpr e(env);
                        e += a[uNode] - (a[i] + tL + tauD[i][j] + tauD[j][uNode]) - M * (1 - yD3[i][j][uNode]);
                        model.add(wD[i][j][uNode] >= e)
                            .setName(("dr_wait_def_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(uNode)).c_str());
                        e.end();
                    }

                    // (33) (τ'ij + τ'ju + tR) * yD3_iju + wD_iju <= E_d
                    {
                        IloExpr e2(env);
                        e2 += (tauD[i][j] + tauD[j][uNode] + tR) * yD3[i][j][uNode] + wD[i][j][uNode];
                        model.add(e2 <= E_d)
                            .setName(("dr_end_case2_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(uNode)).c_str());
                        e2.end();
                    }
                }

        // Truck waiting time must accommodate JetSuite sorties (mode 31)
        for (int i : V)
        {
            IloExpr expr(env);
            for (int k : C)
            {
                // tổng thời gian 1 sortie JetSuite: launch + đi + về + recover
                expr += (2.0 * tauJ[i][k] + tJL + tJR) * yJ[i][k];
            }
            // T[i] >= tổng thời gian JetSuite tại node i
            model.add(T[i] >= expr)
                .setName(("wait_jetsuite_" + std::to_string(i)).c_str());
            expr.end();
        }

        for (int i : V)
        {
            IloExpr sum(env);
            for (int j : C)
                for (int uNode : V)
                    sum += yD3[i][j][uNode];
            model.add(sum <= 1).setName(("dr_launch_cap_" + std::to_string(i)).c_str());
            sum.end();
        }
        // at most 1 retrieve per u
        for (int uNode : V)
        {
            IloExpr sum(env);
            for (int i : V)
                for (int j : C)
                    sum += yD3[i][j][uNode];
            model.add(sum <= 1).setName(("dr_retrieve_cap_" + std::to_string(uNode)).c_str());
            sum.end();
        }
        // can launch/retrieve only if truck visits i/u
        for (int i : V)
        {
            IloExpr out(env);
            for (int v : V)
                out += x[i][v];
            IloExpr sum(env);
            for (int j : C)
                for (int uNode : V)
                    sum += yD3[i][j][uNode];
            model.add(sum <= out).setName(("dr_launch_truck_" + std::to_string(i)).c_str());
            out.end();
            sum.end();
        }
        for (int uNode : V)
        {
            IloExpr in(env);
            for (int v : V)
                in += x[v][uNode];
            IloExpr sum(env);
            for (int i : V)
                for (int j : C)
                    sum += yD3[i][j][uNode];
            model.add(sum <= in).setName(("dr_retrieve_truck_" + std::to_string(uNode)).c_str());
            in.end();
            sum.end();
        }
        // sync at retrieval u: d[u] >= a[i] + tL + τ'ij + τ'ju + tR - M(1 - yD3)
        //                      a[u] >= a[i] + tL + τ'ij + τ'ju      - M(1 - yD3)
        for (int i : V)
            for (int j : C)
                for (int uNode : V)
                {
                    IloExpr e1(env);
                    e1 += a[i] + tL + tauD[i][j] + tauD[j][uNode] + tR - M * (1 - yD3[i][j][uNode]);
                    model.add(d[uNode] >= e1).setName(("dr_sync_d_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(uNode)).c_str());
                    e1.end();

                    IloExpr e2(env);
                    e2 += a[i] + tL + tauD[i][j] + tauD[j][uNode] - M * (1 - yD3[i][j][uNode]);
                    model.add(a[uNode] >= e2).setName(("dr_sync_a_" + std::to_string(i) + "_" + std::to_string(j) + "_" + std::to_string(uNode)).c_str());
                    e2.end();
                }
        // forbid retrieve == launch; forbid serving own launch index
        for (int i : V)
            for (int j : C)
                model.add(yD3[i][j][i] == 0);
        for (int i : V)
            if (C.count(i))
                for (int uNode : V)
                    model.add(yD3[i][i][uNode] == 0);

        // ---- JetSuite ----
        for (int k : C)
        {
            IloExpr expr(env);
            for (int i : V)
                expr += yJ[i][k] * (2 * tauJ[i][k] + tJR);
            model.add(expr <= E_J).setName((std::string("js_end_k") + std::to_string(k)).c_str());
            expr.end();
        }

        for (int k : C)
        {
            model.add(yJ[S][k] == 0).setName("no_jetsuite_at_depot");
        }

        for (int i : V)
        {
            IloExpr e(env);
            for (int k : C)
                e += yJ[i][k];
            model.add(e <= 1).setName(("js_atmost1_i" + std::to_string(i)).c_str());
            e.end();
        }
        for (int i : V)
        {
            IloExpr out(env);
            for (int v : V)
                out += x[i][v];
            for (int k : C)
                model.add(yJ[i][k] <= out).setName((std::string("js_truck_i") + std::to_string(i) + "_k" + std::to_string(k)).c_str());
            out.end();
        }
        {
            IloExpr e(env);
            for (int k : C)
                e += yJ[S][k];
            model.add(e == 0).setName("js_after_depot");
            e.end();
        }
        for (int i : V)
        {
            if (C.count(i))
                model.add(yJ[i][i] == 0);
        }

        // ---- Exactly one service per customer j ----
        for (int j : C)
        {
            IloExpr truck(env), drone(env), jetsuite(env);
            for (int i : V)
                truck += x[i][j];
            for (int i : V)
                for (int uNode : V)
                    drone += yD3[i][j][uNode];
            for (int i : V)
                jetsuite += yJ[i][j];
            model.add(truck + drone + jetsuite == 1).setName((std::string("served_once_case2_") + std::to_string(j)).c_str());
            truck.end();
            drone.end();
            jetsuite.end();
        }

        // ---- Customers must be truck-served (no drone) ----
        for (int h : instance->no_visit)
        {
            for (int i : V)
                for (int uNode : V)
                    model.add(yD3[i][h][uNode] == 0)
                        .setName((std::string("no_drone_case2_") + std::to_string(h)).c_str());
        }

        // ---- Solve ----
        IloCplex cplex(model);
        if (cfg.screen_mode == 0 || cfg.screen_mode == 1)
        {
            cplex.setOut(env.getNullStream());
            cplex.setError(env.getNullStream());
            cplex.setWarning(env.getNullStream());
        }
        if (cfg.tl > 0)
            cplex.setParam(IloCplex::Param::TimeLimit, cfg.tl);
        if (cfg.num_thread > 0)
            cplex.setParam(IloCplex::Param::Threads, cfg.num_thread);

        auto t0 = std::chrono::high_resolution_clock::now();
        cplex.solve();
        auto t1 = std::chrono::high_resolution_clock::now();
        double solve_time = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() / 1000.0;

        double objv = cplex.getObjValue();
        double gap = std::round(cplex.getMIPRelativeGap() * 10000.0) / 100.0;

        // ==============================
        // In lộ trình nếu cần
        // ==============================
        if (cfg.screen_mode >= 1 &&
            (cplex.getStatus() == IloAlgorithm::Optimal ||
             cplex.getStatus() == IloAlgorithm::Feasible))
        {
            using std::cout;
            using std::endl;

            // ---------- Truck route ----------
            cout << "\n=== Truck route (from x_ij) ===\n";
            std::vector<int> next_of(N + 1, -1);

            for (int i : V)
            {
                for (int j : V)
                {
                    if (i == j)
                        continue;
                    if (cplex.isExtracted(x[i][j]) && cplex.getValue(x[i][j]) > 0.5)
                    {
                        next_of[i] = j;
                        cout << "Truck arc: " << i << " -> " << j << "\n";
                    }
                }
            }

            cout << "Truck path: ";
            int cur = S;
            std::unordered_set<int> seen;
            cout << cur;
            while (true)
            {
                int nxt = (cur >= 0 && cur <= N) ? next_of[cur] : -1;
                if (nxt < 0 || seen.count(nxt))
                    break;
                cout << " -> " << nxt;
                seen.insert(nxt);
                if (nxt == S)
                    break; // quay về depot
                cur = nxt;
            }
            cout << "\n";

            // ---------- Drone sorties ----------
            cout << "\n=== Drone sorties (i, j, u) ===\n";
            bool anyDrone = false;
            for (int i : V)
            {
                for (int j : C)
                {
                    for (int uNode : V)
                    {
                        if (cplex.isExtracted(yD3[i][j][uNode]) &&
                            cplex.getValue(yD3[i][j][uNode]) > 0.5)
                        {
                            anyDrone = true;
                            cout << "Drone: launch at " << i
                                 << ", serve customer " << j
                                 << ", rendezvous at " << uNode << "\n";
                        }
                    }
                }
            }
            if (!anyDrone)
                cout << "(no drone sorties)\n";

            // ---------- Jetsuite sorties ----------
            cout << "\n=== Jetsuite sorties (i, k) ===\n";
            bool anyJS = false;
            for (int i : V)
            {
                for (int k : C)
                {
                    if (cplex.isExtracted(yJ[i][k]) &&
                        cplex.getValue(yJ[i][k]) > 0.5)
                    {
                        anyJS = true;
                        cout << "Jetsuite: launch at " << i
                             << ", serve customer " << k
                             << ", return to truck at node " << i << "\n";
                    }
                }
            }
            if (!anyJS)
                cout << "(no jetsuite sorties)\n";
            cout << endl;
        }

        return Result{objv, solve_time, gap};
    }
    catch (IloException &e)
    {
        std::cerr << "Concert exception: " << e << std::endl;
    }
    catch (...)
    {
        std::cerr << "Unknown exception" << std::endl;
    }
    return Result{-1, -1, -1};
}