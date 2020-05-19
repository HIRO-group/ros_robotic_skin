// example: construct a quadratic program from data
// the QP below is the first quadratic program example in the user manual
#include <iostream>
#include <cassert>

#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>

// choose exact integral type
#ifdef CGAL_USE_GMP
#include <CGAL/Gmpz.h>
typedef CGAL::Gmpz ET;
#else
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;
#endif

#include "Eigen/Dense"

class QPAvoidance
{
private:
    CGAL::Quadratic_program<int> qp;
    CGAL::Quadratic_program_solution<ET> solution;
    void EigenSetA(Eigen::MatrixXd A);
    void EigenSetB(Eigen::VectorXd v);
    void EigenSetU(Eigen::VectorXd u);
    void EigenSetD(Eigen::MatrixXd D);
    void EigenSetC(Eigen::VectorXd c);

public:
    QPAvoidance(/* args */);
    ~QPAvoidance();
    void run();
};

QPAvoidance::QPAvoidance(/* args */)
{
    // Non negative solutions by default
    qp = CGAL::Quadratic_program<int>(CGAL::SMALLER, true, 0, false, 0);
}

QPAvoidance::~QPAvoidance()
{
}

void QPAvoidance::EigenSetA(Eigen::MatrixXd A)
{
    A.transposeInPlace();
    for(int i=0; i < A.rows(); i++)
    {
        for(int j=0; j<A.cols(); j++)
            qp.set_a(i, j,  A(i,j));
    }
}

void QPAvoidance::EigenSetB(Eigen::VectorXd v)
{
    for(int i=0; i < v.size(); i++)
    {
        qp.set_b(i, v(i));
    }
}

void QPAvoidance::EigenSetU(Eigen::VectorXd u)
{
    for(int i=0; i<u.size(); i++)
        if (u(i) != Eigen::Infinity){qp.set_u(i, true, u(i));}
}

void QPAvoidance::EigenSetD(Eigen::MatrixXd D)
{
    D.transposeInPlace();
    for(int i=0; i < D.rows(); i++)
    {
        for(int j=0; j<D.cols(); j++)
            qp.set_d(i, j,  D(i,j));
    }
}

void QPAvoidance::EigenSetC(Eigen::VectorXd c)
{
    for(int i=0; i<c.size(); i++)
        qp.set_c(i, c(i));
}

void QPAvoidance::run()
{
    // now set the non-default entries:
    const int X = 0;
    const int Y = 1;

    Eigen::Matrix2d A; A << 1, 1,
                            -1, 2;
    Eigen::Vector2d b; b << 7, 4;
    Eigen::Vector2d u; u << Eigen::Infinity, 4;
    Eigen::Matrix2d D; D << 2, 0,
                            0, 8;
    Eigen::Vector2d c; c << 0, -32;

    EigenSetA(A);
    EigenSetB(b);
    EigenSetU(u);
    EigenSetD(D);
    EigenSetC(c);

    // solve the program, using ET as the exact type
    solution = CGAL::solve_quadratic_program(qp, ET());
    assert (solution.solves_quadratic_program(qp));
    std::cout << solution;
    std::cout << solution.number_of_iterations() << std::endl;
    std::cout << solution.is_infeasible() << std::endl;
    for (CGAL::Quadratic_program_solution<ET>::Variable_value_iterator it = solution.variable_values_begin(); it < solution.variable_values_end(); ++it)
    {

        std::cout << CGAL::to_double(*it) << std::endl;
    }




}

int main()
{
    QPAvoidance qpAvoidance;
    qpAvoidance.run();
    return 0;
}