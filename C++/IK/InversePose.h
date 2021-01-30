#ifndef INVERSE_POSE_H
#define INVERSE_POSE_H

#include <vector>
// #include <Eigen/Dense>

class InversePose
{
private:
    std::vector<float> thetas;
    std::vector<float> as;
    std::vector<float> ds;
public:
    InversePose(std::vector<float> robotGeometryAs, std::vector<float> robotGeometryDs);
};

InversePose::InversePose(std::vector<float> robotGeometryAs, std::vector<float> robotGeometryDs)
{
    as = robotGeometryAs;
    ds = robotGeometryDs;
    for (int i=0; i<6; i++)
    {
        thetas[i] = 0;
    }
}


#endif // INVERSE_POSE_H