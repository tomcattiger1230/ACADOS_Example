//
// Created by Wei Luo on 2021/3/18.
//

#include "mobile_robot_app.h"

int main()
{
    // create a capsule according to the pre-defined model
    mobile_robot_solver_capsule *acados_ocp_capsule = mobile_robot_acados_create_capsule();

    // optimizer
    status = mobile_robot_acados_create(acados_ocp_capsule);

    if (status)
    {
        printf("mobile_robot_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }
    sim_solver_capsule *sim_capsule = mobile_robot_acados_sim_solver_create_capsule();
    status = mobile_robot_acados_sim_create(sim_capsule);
    sim_config *mobile_robot_sim_config = mobile_robot_acados_get_sim_config(sim_capsule);
    void *mobile_robot_sim_dims = mobile_robot_acados_get_sim_dims(sim_capsule);
    sim_in *mobile_robot_sim_in = mobile_robot_acados_get_sim_in(sim_capsule);
    sim_out *mobile_robot_sim_out = mobile_robot_acados_get_sim_out(sim_capsule);

    if (status)
    {
        printf("acados_create() simulator returned status %d. Exiting.\n", status);
        exit(1);
    }

    // some important structure of ocp
    ocp_nlp_config *nlp_config = mobile_robot_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims = mobile_robot_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = mobile_robot_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out = mobile_robot_acados_get_nlp_out(acados_ocp_capsule);
    //    ocp_nlp_solver *nlp_solver = mobile_robot_acados_get_nlp_solver(acados_ocp_capsule);
    //    void *nlp_opts = mobile_robot_acados_get_nlp_opts(acados_ocp_capsule);

    N = nlp_dims->N;
    nx = *nlp_dims->nx;
    nu = *nlp_dims->nu;
    printf("time horizion is %d, with state %d and input %d \n", N, nx, nu);

    Eigen::MatrixXd simX((N + 1), nx);
    Eigen::MatrixXd simU(N, nu);
    Eigen::VectorXd time_record(N);

    x_current[0] = 0.0;
    x_current[1] = 0.0;
    x_current[2] = 0.0;

    x_target[0] = 2.0;
    x_target[1] = 2.0;
    x_target[2] = 1.0;

    x_state[0] = 2.0;
    x_state[1] = 2.0;
    x_state[2] = 1.0;
    x_state[3] = 0.0;
    x_state[4] = 0.0;

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", x_target);

    for (int i = 0; i < nx; i++)
        simX(0, i) = x_current[i];

    for (int i = 0; i < N; i++)
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", x_state);

    // closed loop simulation
    for (int ii = 0; ii < N; ii++)
    {
        auto t_start = std::chrono::high_resolution_clock::now();
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", x_current);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", x_current);

        status = mobile_robot_acados_solve(acados_ocp_capsule);
        //        ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
        //        min_time = MIN(elapsed_time, min_time);

        // get the optimized control input
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", &u_current);
        for (int i = 0; i < nu; i++)
            simU(ii, i) = u_current[i];

        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        time_record(ii) = elapsed_time_ms;

        // simulation
        sim_in_set(mobile_robot_sim_config, mobile_robot_sim_dims,
                   mobile_robot_sim_in, "u", u_current);
        sim_in_set(mobile_robot_sim_config, mobile_robot_sim_dims,
                   mobile_robot_sim_in, "x", x_current);

        status = mobile_robot_acados_sim_solve(sim_capsule);
        if (status != ACADOS_SUCCESS)
        {
            printf("acados_solve() failed with status %d.\n", status);
        }

        sim_out_get(mobile_robot_sim_config, mobile_robot_sim_dims,
                    mobile_robot_sim_out, "x", x_current);

        for (int i = 0; i < nx; i++)
            simX(ii + 1, i) = x_current[i];
    }

    // print results
    for (int i = 0; i < N + 1; i++)
        printf("Final result index %d %f, %f, %f \n", i, simX(i, 0), simX(i, 1), simX(i, 2));

    printf("average estimation time %f ms \n", time_record.mean());
    printf("max estimation time %f ms \n", time_record.maxCoeff());
    printf("min estimation time %f ms \n", time_record.minCoeff());
    return status;
}