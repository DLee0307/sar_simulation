
#ifndef GZ_SIM_SYSTEMS_CONTACT_HH_
#define GZ_SIM_SYSTEMS_CONTACT_HH_

#include <memory>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class ContactPrivate;

  /** \class Contact Contact.hh \
   * gz/sim/systems/Contact/Contact.hh
  **/
  /// \brief Contact sensor system which manages all contact sensors in
  /// simulation
  class Contact final:
    public System,
    public ISystemPreUpdate,
    public ISystemPostUpdate
  {
    /// \brief Constructor
    public: Contact();

    /// \brief Destructor
    public: ~Contact() final = default;

    /// Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) final;

    /// Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) final;

    /// \brief Private data pointer
    private: std::unique_ptr<ContactPrivate> dataPtr;
  };
}
}
}  // namespace sim
}  // namespace gz
#endif