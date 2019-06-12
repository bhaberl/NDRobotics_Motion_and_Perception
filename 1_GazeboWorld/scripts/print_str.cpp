#include <gazebo/gazebo.hh>

namespace gazebo
{
    class WorldPluginMyRobot : public WorldPlugin
    {
        public: WorldPluginMyRobot() : WorldPlugin()
	    {
		printf("Welcome to my World!\n");
	    {

	public: void Load(physics::WorldPtr_world, sdf::ElementPtr_sdf)
	    {
	    }
    };
    GZ_REGISTER_WORLD_PLUGIN(WorldPluginMyRobot)
}
