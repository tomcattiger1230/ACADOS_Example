//
// Created by Wei Luo on 2021/3/18.
//

#include "mobile_robot_app.h"

int main()
{
    // create a capsule according to the pre-defined model
    nlp_solver_capsule *acados_ocp_capsule = mobile_robot_acados_create_capsule();

    // optimizer
    status = mobile_robot_acados_create(acados_ocp_capsule);

    if (status)
    {
        printf("mobile_robot_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    status = mobile_robot_acados_sim_create();

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
    ocp_nlp_solver *nlp_solver = mobile_robot_acados_get_nlp_solver(acados_ocp_capsule);
    void *nlp_opts = mobile_robot_acados_get_nlp_opts(acados_ocp_capsule);


//    // initial condition
//    idxbx0[0] = 0;
//    idxbx0[1] = 1;
//    idxbx0[2] = 2;
//
//    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);


    int N = nlp_dims->N;
    int nx = *nlp_dims->nx;
    int nu = *nlp_dims->nu;

    printf("N is %d", N);
    Eigen::MatrixXd simX((N+1), nx);
    Eigen::MatrixXd simU(N, nu);

    double xtraj[nx*(N+1)];
    double utraj[nu*N];

    x_current[0] = 0.0;
    x_current[1] = 2.0;
    x_current[2] = 0.0;

    x_target[0] = 1.0;
    x_target[1] = 3.0;
    x_target[2] = 1.0;

    x_state[0] = 1.0;
    x_state[1] = 3.0;
    x_state[2] = 1.0;
    x_state[3] = 0.0;
    x_state[4] = 0.0;

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", x_target);

    for(int i=0; i<nx; i++)
        simX(0, i)  = x_current[i];

    for (int i=0; i<N; i++)
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", x_state);

    // closed loop simulation
    for (int ii=0; ii<1; ii++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", x_current);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", x_current);

        status = mobile_robot_acados_solve(acados_ocp_capsule);
        ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
        min_time = MIN(elapsed_time, min_time);

        double x_value[3];
        for (int k = 0; k <= N; k++) {

            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, k, "x", &x_value); //xtraj[k*3]
            printf("test %f, %f, %f, \n", x_value[0], x_value[1], x_value[2]);
        }
        double u_value[2];
        for (int k = 0; k < nlp_dims->N; k++)
        {
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, k, "u", &u_value); // utraj[k*2]
            printf("test %f, %f, \n", u_value[0], u_value[1]);
        }


//        printf("\n--- xtraj ---\n");
//        d_print_exp_tran_mat( 3, 100+1, xtraj, 3 );
//        printf("\n--- utraj ---\n");
//        d_print_exp_tran_mat( 2, 100, utraj, 2 );
    }


    return status;
}