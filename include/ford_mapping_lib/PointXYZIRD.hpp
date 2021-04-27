#include <pcl/point_types.h>
#include <pcl/impl/pcl_base.hpp>

struct PointXYZIRD{
    PCL_ADD_POINT4D;

    union{
        float data_c[4];
        struct{
            int ring;
            float intensity;
            float distance;
        };
    };
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRD,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (int, ring, ring)
    (float, intensity, intensity)
    (float, distance, distance)
)