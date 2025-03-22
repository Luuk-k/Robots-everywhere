#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/entity_wrench.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/math/Vector3.hh>
#include <iostream>
#include <string>

namespace gz_sim = gz::sim;

class ForceSubscriberPlugin : public gz_sim::System,
                              public gz_sim::ISystemConfigure,
                              public gz_sim::ISystemPreUpdate
{
private:
    gz::transport::Node node;
    gz_sim::Model model{gz_sim::kNullEntity};
    gz_sim::Entity linkEntity{gz_sim::kNullEntity};
    gz::math::Vector3d appliedForce;

public:
    ForceSubscriberPlugin() {}

    void Configure(const gz_sim::Entity &_entity,
               const std::shared_ptr<const sdf::Element> &_sdf,
               gz_sim::EntityComponentManager &_ecm,
               gz_sim::EventManager &) override
{
    // Check if the entity is valid
    if (_entity == gz_sim::kNullEntity)
    {
        std::cerr << "Error: Entity is null!" << std::endl;
        return;
    }
 
    // Store model reference
    this->model = gz_sim::Model(_entity);
    //if (!this->model.Valid(_ecm))
    //{
    //   std::cerr << "Error: Model entity is invalid."<< model.Name(_ecm) << std::endl;
    //    return;
    //}

    // Debug: Output the entity type and ID
    std::cout << "Entity ID: " << _entity << std::endl;

    // Get the link name from SDF
    std::string linkName = "body"; // Default link
    if (_sdf->HasElement("link_name"))
    {
        linkName = _sdf->Get<std::string>("link_name");
    }
    std::cout << "Searching for link: " << linkName << std::endl;

    // Find the link entity using component-based iteration
    auto links = _ecm.ChildrenByComponents(this->model.Entity(), gz_sim::components::Link());
    for (const auto &link : links)
    {
        gz_sim::Link tmpLink(link);

        std::cout << "kut" << std::endl;
        // Check link name by comparing it to the linkName passed from SDF
        std::string _name = tmpLink.Name(_ecm).value_or("empty");
        std::cout << _name << std::endl;
        if (tmpLink.Name(_ecm) == linkName)
       {
            this->linkEntity = link;
            break;
        }
    }

    if (this->linkEntity == gz_sim::kNullEntity)
    {
        std::cerr << "Error: Could not find link [" << linkName << "]" << std::endl;
        return;
    }

    std::cout << "Successfully found model and link. Subscribing to wrench topic..." << std::endl;

    // ✅ Corrected subscription method (no SubscriptionHandler)
    this->node.Subscribe("/world/world_test/wrench",
                         &ForceSubscriberPlugin::OnWrenchReceived, this);
}


    void OnWrenchReceived(const gz::msgs::EntityWrench &msg)
    {
        if (!msg.has_wrench())
        {
            std::cerr << "Error: Received message without wrench data!" << std::endl;
            return;
        }

        // ✅ Convert gz::msgs::Vector3d to gz::math::Vector3d
        const gz::msgs::Vector3d &forceMsg = msg.wrench().force();
        this->appliedForce = gz::math::Vector3d(forceMsg.x(), forceMsg.y(), forceMsg.z());

        std::cout << "Received force: " << this->appliedForce << std::endl;
    }

    void PreUpdate(const gz_sim::UpdateInfo &, gz_sim::EntityComponentManager &_ecm) override
    {
        if (this->linkEntity == gz_sim::kNullEntity)
            return;

        // Apply force using ExternalWorldWrenchCmd component
        auto *wrenchComp = _ecm.Component<gz_sim::components::ExternalWorldWrenchCmd>(this->linkEntity);

        // Create the component if it doesn't exist
        if (!wrenchComp)
        {
            // Correct way to create the component without ambiguity
            _ecm.CreateComponent(this->linkEntity, gz_sim::components::ExternalWorldWrenchCmd());
            wrenchComp = _ecm.Component<gz_sim::components::ExternalWorldWrenchCmd>(this->linkEntity);
        }

        if (wrenchComp)
        {
            // Use mutable_force() to set force values correctly
            auto *force = wrenchComp->Data().mutable_force();  // Use mutable_force()

            // Set force components
            force->set_x(this->appliedForce.X());
            force->set_y(this->appliedForce.Y());
            force->set_z(this->appliedForce.Z());

            _ecm.SetChanged(this->linkEntity, gz_sim::components::ExternalWorldWrenchCmd::typeId, gz_sim::ComponentState::OneTimeChange);
        }
    }
};

// Register the plugin
GZ_ADD_PLUGIN(ForceSubscriberPlugin, gz_sim::System,
              ForceSubscriberPlugin::ISystemConfigure,
              ForceSubscriberPlugin::ISystemPreUpdate)

