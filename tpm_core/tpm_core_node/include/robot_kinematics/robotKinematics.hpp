#include <vector>
#include <string>
#include "MCL_Types.h"
#include "ROB_DEF.h"

namespace tpm_core
{
  class RobotKinematics //this is a singleton class.
  {
  protected:
    // private constructor, copy constructor, and assignment operator
    RobotKinematics(){};
    RobotKinematics(const RobotKinematics&);
    RobotKinematics& operator=(const RobotKinematics&);

  public:
    // RobotKinematics(const RobotKinematics&)=delete;
    // RobotKinematics& operator=(const RobotKinematics&)=delete;
    // ~RobotKinematics();// This will be automatically called when the program exits

    virtual void Init(FLT a[], FLT alpha[], FLT d[], FLT theta[]) = 0;

    virtual void GetJointStates(FLT* axes, FLT* pos, std::vector<std::string>& jointNames, std::vector<double>& jointValues){}

  protected:
    std::vector<std::string> _joint_names;

  };

  class DeltaKinematics : public RobotKinematics
  {
  public:
    static DeltaKinematics& Instance()
    {
      static DeltaKinematics instance; //initialized only once.
      return instance;
    }
    void Init(FLT a[], FLT alpha[], FLT d[], FLT theta[]) override;
    void GetJointStates(FLT* axes, FLT* pos, std::vector<std::string>& jointNames, std::vector<double>& jointValues) override;

  private:
    FLT phi[3];
    FLT theta; // slide angle
    FLT R;     // radius of top plane
    FLT r;     // radius of bottom plane
    FLT l_shift;
    FLT L;
    FLT H;
  };
} // namespace tpm_core


