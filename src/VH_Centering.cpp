#include <DDImage/Knob.h>
#include <DDImage/Knobs.h>
#include <DDImage/ParticleOp.h>
#include <DDImage/Vector3.h>

using namespace DD::Image;

namespace vh {

  //
  // Constants
  //

  static const char* kPluginClass = "VH_Centering";
  static const char* kPluginHelp = "Particles will try to move towards the centre of the flock.";


  //
  // VH_Centering class
  //

  class VH_Centering : public ParticleBehaviour {
  public:
    VH_Centering(Node* node);
    virtual ~VH_Centering() {}

    virtual const char* Class() const { return kPluginClass; }
    virtual const char* node_help() const { return kPluginHelp; }

    virtual void knobs(Knob_Callback f);
#if NUKE_VERSION_MAJOR >= 7
    virtual bool applyBehaviour(const ParticleContext& context, ParticleSystem* ps);
#else
    virtual void applyBehaviour(const ParticleContext& context, ParticleSystem* ps);
#endif

    static const Description desc;

  private:
    void centering(const ParticleContext& context, ParticleSystem* ps);

  private:
    float _centeringRate;
  };


  //
  // VH_Centering methods
  //

  VH_Centering::VH_Centering(Node* node) :
    ParticleBehaviour(node),
    _centeringRate(100.0f)
  {
  }


  void VH_Centering::knobs(Knob_Callback f)
  {
    ParticleBehaviour::knobs(f);
    Float_knob(f, &_centeringRate, "centering_rate", "centering rate");
  }


#if NUKE_VERSION_MAJOR >= 7
  bool VH_Centering::applyBehaviour(const ParticleContext& context, ParticleSystem* ps)
  {
    centering(context, ps);
    return true;
  }
#else
  void VH_Centering::applyBehaviour(const ParticleContext& context, ParticleSystem* ps)
  {
    centering(context, ps);
  }
#endif


  void VH_Centering::centering(const ParticleContext& context, ParticleSystem* ps)
  {
    const unsigned int kNumParticles = ps->numParticles();
    const double kEnd = context.endTime();

    // Pre-calculations
    Vector3 summedPositions(0, 0, 0);
    for (unsigned int i = 0; i < kNumParticles; ++i) {
      summedPositions += ps->particlePosition(i);
    }

    // Apply the flocking rules to each particle.
    const unsigned int kNumOtherParticles = (kNumParticles > 1) ? (kNumParticles - 1) : 1;
    for (unsigned int i = 0; i < kNumParticles; ++i) {
      double startTime = ps->particleStartTime(i);
      if (startTime > kEnd)
        continue;

      double dt = kEnd - startTime;
      if (dt > context.dt())
        dt = context.dt();

      Vector3 position = ps->particlePosition(i);
      Vector3 perceivedCenter = (summedPositions - position) / kNumOtherParticles;
      Vector3 motionTowardsCenter = (perceivedCenter - position) / _centeringRate;

      // Apply the total force to the particle.
      applyForce(ps, i, context, motionTowardsCenter);
    }
  }


  //
  // Functions
  //

  static Op* MakeOp(Node* node)
  {
    return new VH_Centering(node);
  }


  //
  // Static members
  //

  const Op::Description VH_Centering::desc(kPluginClass, kPluginHelp, MakeOp);

} // namespace vh

