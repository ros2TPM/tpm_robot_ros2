#include <vector>
#include <string>
#include "MCL_Types.h"


#ifdef ROB_REAL
  #include "RPiRobIF.h"
#else
  #include "ROB_DEF.h"
#endif

namespace tpm_core
{
  class RobotKinematics //this is a singleton class.
  {
  protected:
    // private constructor, copy constructor, and assignment operator
    RobotKinematics(){}

  public:
    static void SelectKinematic(ROB_KIN_TYPE type);
    static RobotKinematics* GetInstance() { return _ptr; }
    RobotKinematics(const RobotKinematics&)=delete;
    RobotKinematics& operator=(const RobotKinematics&)=delete;
    virtual ~RobotKinematics(){ delete(_ptr); } // This will be automatically called when the program exits

    virtual void Init(FLT a[], FLT alpha[], FLT d[], FLT theta[]) = 0;

    virtual void GetJointStates(FLT* axes, FLT* pos, std::vector<std::string>& jointNames, std::vector<double>& jointValues){}

  protected:
    static RobotKinematics* _ptr;
    std::vector<std::string> _joint_names;

  };

  class SerialKinematics : public RobotKinematics
  {
    void Init(FLT a[], FLT alpha[], FLT d[], FLT theta[]) override;
    void GetJointStates(FLT* axes, FLT* pos, std::vector<std::string>& jointNames, std::vector<double>& jointValues) override;
  };

  class DeltaKinematics : public RobotKinematics
  {
  public:
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


