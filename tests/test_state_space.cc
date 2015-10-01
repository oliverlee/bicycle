#include <Eigen/Dense>
#include "gtest/gtest.h"
#include "bicycle.h"
#include "parameters.h"
#include "test_utilities.h"
#include "test_state_space.h"

/*
 * A and B expected state space matrices generated using dtk.bicycle:
 *
 * from dtk.bicycle import *
 * np.set_printoptions(precision=16)
 * A, B = benchmark_state_space(*benchmark_matrices(), v=1, g=9.80665)
 *
 * Ad, Bd generated using scipy.signal
 *
 * import scipy.signal as sig
 * sig.cont2discrete((A, B, np.eye(4), np.zeros((4, 2))), 1/200)
 */


namespace {
    const double dt = 1.0/200;
    Eigen::Matrix<double, 4, 4> A;
    Eigen::Matrix<double, 4, 2> B(
        (Eigen::Matrix<double, 4, 2>() <<
            0.                ,  0.                ,
            0.                ,  0.                ,
            0.0159349789179135, -0.1240920254115741,
           -0.1240920254115741,  4.3238401808042282).finished()
        );
    Eigen::Matrix<double, 4, 4> Ad;
    Eigen::Matrix<double, 4, 2> Bd;

    const double vw = 4.29238253634111; // forward speed [m/s]
    const double vc = 6.02426201538837; // forward speed [m/s]

    // These matrices are (obviously) not correct and are used only to
    // determine if discrete state space matrices are correctly looked up.
    const model::Bicycle::state_matrix_t Ad_vw(
            2 * model::Bicycle::state_matrix_t::Identity()
            );
    const model::Bicycle::input_matrix_t Bd_vw(
            3 * model::Bicycle::input_matrix_t::Identity()
            );
    const model::Bicycle::state_matrix_t Ad_vc(
            4 * model::Bicycle::state_matrix_t::Identity()
            );
    const model::Bicycle::input_matrix_t Bd_vc(
            5 * model::Bicycle::input_matrix_t::Identity()
            );

    const model::Bicycle::state_space_map_t state_space_map {
        {model::Bicycle::make_state_space_map_key(vw, dt),
            model::Bicycle::state_space_map_value_t(Ad_vw, Bd_vw)},
        {model::Bicycle::make_state_space_map_key(vc, dt),
            model::Bicycle::state_space_map_value_t(Ad_vc, Bd_vc)},
    };
} // namespace

TEST_F(StateSpaceTest, ContinuousV1) {
    bicycle->set_v(1.0, 0);

    A <<  0.                ,   0.                ,   1.                , 0.                ,
          0.                ,   0.                ,   0.                , 1.                ,
          9.4865338000460664,  -1.4625257433243051,  -0.1055224498056882, -0.330515398992312 ,
         11.7154748079957685,  28.9264833312917631,   3.6768052333214327, -3.0848655274330694;

    EXPECT_TRUE(bicycle->A().isApprox(A)) << output_matrices(bicycle->A(), A);
    EXPECT_TRUE(bicycle->B().isApprox(B)) << output_matrices(bicycle->B(), B);
}

TEST_F(StateSpaceTest, ContinuousV3) {
    bicycle->set_v(3.0, 0);

    A <<  0.                ,   0.                ,   1.                , 0.                ,
          0.                ,   0.                ,   0.                , 1.                ,
          9.4865338000460664,  -8.5921076477970253,  -0.3165673494170646, -0.9915461969769359,
         11.7154748079957685,  13.1527626512942426,  11.0304156999642977, -9.2545965822992091;

    EXPECT_TRUE(bicycle->A().isApprox(A)) << output_matrices(bicycle->A(), A);
    EXPECT_TRUE(bicycle->B().isApprox(B)) << output_matrices(bicycle->B(), B);
}

TEST_F(StateSpaceTest, ContinuousV5) {
    bicycle->set_v(5.0, 0);

    A <<  0.                ,   0.                ,   1.                , 0.                ,
          0.                ,   0.                ,   0.                , 1.                ,
          9.4865338000460664, -22.8512714567424666,  -0.5276122490284411, -1.6525769949615603,
         11.7154748079957685, -18.3946787087007344,  18.384026166607164 , -15.4243276371653479;

    EXPECT_TRUE(bicycle->A().isApprox(A)) << output_matrices(bicycle->A(), A);
    EXPECT_TRUE(bicycle->B().isApprox(B)) << output_matrices(bicycle->B(), B);
}

TEST_F(StateSpaceTest, DiscreteV1) {
    bicycle->set_v(1.0, dt);

    Ad << 1.0001184820643081e+00,  -1.8478167519170527e-05, 4.9988533321204658e-03,  -4.1402267568149167e-06,
          1.4642849817488363e-04,   1.0003596378458957e+00, 4.5963276543359894e-05,   4.9622093457528903e-03,
          4.7373286374364838e-02,  -7.4307138855974368e-03, 9.9957576800707704e-01,  -1.6579041282911602e-03,
          5.8570670758658606e-02,   1.4347204345110903e-01, 1.8386655631933688e-02,   9.8503669772459101e-01;
    Bd << 2.0001145816138571e-07,  -1.5807242572795020e-06,
         -1.5420741274461165e-06,   5.3764780115010109e-05,
          8.0170391584997460e-05,  -6.3821951352698188e-04,
         -6.1503818438800187e-04,   2.1450096478647790e-02;

    EXPECT_TRUE(bicycle->Ad().isApprox(Ad)) << output_matrices(bicycle->Ad(), Ad);
    EXPECT_TRUE(bicycle->Bd().isApprox(Bd)) << output_matrices(bicycle->Bd(), Bd);
}

TEST_F(StateSpaceTest, DiscreteV3) {
    bicycle->set_v(3.0, dt);

    Ad << 1.0001182770323798e+00,  -1.0761577382854053e-04, 4.9960146578785398e-03,  -1.2376061665969063e-05,
          1.4636823146804625e-04,   1.0001599497146791e+00, 1.3595045476592029e-04,   4.8861238841920599e-03,
          4.7249870478820497e-02,  -4.3089075152114520e-02, 9.9840018880958248e-01,  -4.9468596498928336e-03,
          5.8532959858267050e-02,   6.3097926791482337e-02, 5.3999308360513393e-02,   9.5480604315894413e-01;
    Bd << 2.0164212892573775e-07,  -1.6395468174733748e-06,
         -1.5238824775089880e-06,   5.3195920730699685e-05,
          8.1147158805629875e-05,  -6.7347769059348866e-04,
         -6.0416264157068470e-04,   2.1109948411569327e-02;

    EXPECT_TRUE(bicycle->Ad().isApprox(Ad)) << output_matrices(bicycle->Ad(), Ad);
    EXPECT_TRUE(bicycle->Bd().isApprox(Bd)) << output_matrices(bicycle->Bd(), Bd);
}

TEST_F(StateSpaceTest, DiscreteV5) {
    bicycle->set_v(5.0, dt);

    Ad << 1.0001184820643081e+00,  -1.8478167519170527e-05, 4.9988533321204658e-03,  -4.1402267568149167e-06,
          1.4642849817488363e-04,   1.0003596378458957e+00, 4.5963276543359894e-05,   4.9622093457528903e-03,
          4.7373286374364838e-02,  -7.4307138855974368e-03, 9.9957576800707704e-01,  -1.6579041282911602e-03,
          5.8570670758658606e-02,   1.4347204345110903e-01, 1.8386655631933688e-02,   9.8503669772459101e-01;
    Bd << 2.0001145816138571e-07,  -1.5807242572795020e-06,
         -1.5420741274461165e-06,   5.3764780115010109e-05,
          8.0170391584997460e-05,  -6.3821951352698188e-04,
         -6.1503818438800187e-04,   2.1450096478647790e-02;
    Ad << 1.0001180700462438e+00,  -2.8474586368268200e-04, 4.9929766799901975e-03,  -2.0583494132583435e-05,
          1.4630038234223096e-04,   9.9976730145466564e-01, 2.2402776466154753e-04,   4.8110697443882302e-03,
          4.7124896630597990e-02,  -1.1371723873036946e-01, 9.9710530689603383e-01,  -8.2185377039953947e-03,
          5.8489213351501479e-02,  -9.3617401457300686e-02, 8.8474932659789590e-02,   9.2518956230185589e-01;
    Bd << 2.0326445533610386e-07,  -1.6981861891088082e-06,
         -1.5058897428593093e-06,   5.2632958211780904e-05,
          8.2117225610236940e-05,  -7.0858832804455301e-04,
         -5.9344551127057076e-04,   2.0774496614372077e-02;

    EXPECT_TRUE(bicycle->Ad().isApprox(Ad)) << output_matrices(bicycle->Ad(), Ad);
    EXPECT_TRUE(bicycle->Bd().isApprox(Bd)) << output_matrices(bicycle->Bd(), Bd);
}

TEST(StateSpace, LookupFound) {
    model::Bicycle bicycle0(parameters::benchmark::M, parameters::benchmark::C1,
            parameters::benchmark::K0, parameters::benchmark::K2,
            parameters::benchmark::wheelbase,
            parameters::benchmark::trail,
            parameters::benchmark::steer_axis_tilt,
            vw, dt, &state_space_map);
    model::Bicycle bicycle1(parameters::benchmark::M, parameters::benchmark::C1,
            parameters::benchmark::K0, parameters::benchmark::K2,
            parameters::benchmark::wheelbase,
            parameters::benchmark::trail,
            parameters::benchmark::steer_axis_tilt,
            vw, dt);

    EXPECT_FALSE(bicycle0.Ad().isApprox(bicycle1.Ad())) << output_matrices(bicycle0.Ad(), bicycle1.Ad());
    EXPECT_FALSE(bicycle0.Bd().isApprox(bicycle1.Bd())) << output_matrices(bicycle0.Bd(), bicycle1.Bd());
    EXPECT_TRUE(bicycle0.Ad().isApprox(Ad_vw)) << output_matrices(bicycle0.Ad(), Ad_vw);
    EXPECT_TRUE(bicycle0.Bd().isApprox(Bd_vw)) << output_matrices(bicycle0.Bd(), Bd_vw);

    bicycle0.set_v(vc, dt);
    bicycle1.set_v(vc, dt);

    EXPECT_FALSE(bicycle0.Ad().isApprox(bicycle1.Ad())) << output_matrices(bicycle0.Ad(), bicycle1.Ad());
    EXPECT_FALSE(bicycle0.Bd().isApprox(bicycle1.Bd())) << output_matrices(bicycle0.Bd(), bicycle1.Bd());
    EXPECT_TRUE(bicycle0.Ad().isApprox(Ad_vc)) << output_matrices(bicycle0.Ad(), Ad_vc);
    EXPECT_TRUE(bicycle0.Bd().isApprox(Bd_vc)) << output_matrices(bicycle0.Bd(), Bd_vc);
}

TEST(StateSpace, LookupNotFound) {
    double v = 1.0;
    model::Bicycle bicycle0(parameters::benchmark::M, parameters::benchmark::C1,
            parameters::benchmark::K0, parameters::benchmark::K2,
            parameters::benchmark::wheelbase,
            parameters::benchmark::trail,
            parameters::benchmark::steer_axis_tilt,
            v, dt, &state_space_map);
    model::Bicycle bicycle1(parameters::benchmark::M, parameters::benchmark::C1,
            parameters::benchmark::K0, parameters::benchmark::K2,
            parameters::benchmark::wheelbase,
            parameters::benchmark::trail,
            parameters::benchmark::steer_axis_tilt,
            v, dt);

    EXPECT_TRUE(bicycle0.Ad().isApprox(bicycle1.Ad())) << output_matrices(bicycle0.Ad(), bicycle1.Ad());
    EXPECT_TRUE(bicycle0.Bd().isApprox(bicycle1.Bd())) << output_matrices(bicycle0.Bd(), bicycle1.Bd());

    v = 5.0;
    bicycle0.set_v(v, dt);
    bicycle1.set_v(v, dt);

    EXPECT_TRUE(bicycle0.Ad().isApprox(bicycle1.Ad())) << output_matrices(bicycle0.Ad(), bicycle1.Ad());
    EXPECT_TRUE(bicycle0.Bd().isApprox(bicycle1.Bd())) << output_matrices(bicycle0.Bd(), bicycle1.Bd());
}
