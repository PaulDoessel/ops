
#include <iomanip>
#include <sstream>
#include "OpenSteer/PolylineSegmentedPathwaySingleRadius.h"
#include "OpenSteer/SimpleVehicle.h"
#include "OpenSteer/OpenSteerDemo.h"
#include "OpenSteer/Proximity.h"
#include "OpenSteer/Color.h"

#include "steer_world.h"

namespace ops
{
    using namespace OpenSteer;

    // ----------------------------------------------------------------------------
    typedef AbstractProximityDatabase<AbstractVehicle*> ProximityDatabase;
    typedef AbstractTokenForProximityDatabase<AbstractVehicle*> ProximityToken;
    // ----------------------------------------------------------------------------

    class pedestrian : public SimpleVehicle
    {
    public:
        // type for a group of Pedestrians
        typedef std::vector<pedestrian*> groupType;

        pedestrian (ProximityDatabase& pd, steer_world* steer_world_ptr,PedestrianConfig config);
        virtual ~pedestrian ();

        void reset (PedestrianConfig config);
        void update (const float currentTime, const float elapsedTime);
        Vec3 determineCombinedSteering (const float elapsedTime);
        // switch to new proximity database -- just for demo purposes
        void newPD (ProximityDatabase& pd);

        // a pointer to this boid's interface object for the proximity database
        ProximityToken* proximityToken;

        // allocate one and share amoung instances just to save memory usage
        // (change to per-instance allocation to be more MP-safe)
        static AVGroup neighbors;

        PolylineSegmentedPathwaySingleRadius* path;

        // direction for path following (upstream or downstream)
        int pathDirection;
        int pathIndex;
        ops::steer_world* mySteerWorldPtr;
        float wander;
    };

} // anonymous namespace
