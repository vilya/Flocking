#include <DDImage/Knob.h>
#include <DDImage/Knobs.h>
#include <DDImage/ParticleOp.h>
#include <DDImage/Vector3.h>

using namespace DD::Image;

namespace vh {

  //
  // Constants
  //

  static const char* kPluginClass = "VH_Separation";
  static const char* kPluginHelp = "Apply simple rules to particles, to make them behave like a flock.";


  //
  // VH_Separation class
  //

  class VH_Separation : public ParticleBehaviour {
  public:
    VH_Separation(Node* node);
    virtual ~VH_Separation() {}

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
    void separation(const ParticleContext& context, ParticleSystem* ps);

  private:
    float _separation;
  };


  //
  // VH_Separation methods
  //

  VH_Separation::VH_Separation(Node* node) :
    ParticleBehaviour(node),
    _separation(0.1f)
  {
  }


  void VH_Separation::knobs(Knob_Callback f)
  {
    ParticleBehaviour::knobs(f);
    Float_knob(f, &_separation, "separation_distance", "separation distance");
  }


#if NUKE_VERSION_MAJOR >= 7
  bool VH_Separation::applyBehaviour(const ParticleContext& context, ParticleSystem* ps)
  {
    separation(context, ps);
    return true;
  }
#else
  void VH_Separation::applyBehaviour(const ParticleContext& context, ParticleSystem* ps)
  {
    separation(context, ps);
  }
#endif


  void VH_Separation::separation(const ParticleContext& context, ParticleSystem* ps)
  {
    const unsigned int kNumParticles = ps->numParticles();
    const float kSeparationSquared = _separation * _separation;
    const double kEnd = context.endTime();

    // Apply the flocking rules to each particle.
    for (unsigned int i = 0; i < kNumParticles; ++i) {
      double startTime = ps->particleStartTime(i);
      if (startTime > kEnd)
        continue;

      double dt = kEnd - startTime;
      if (dt > context.dt())
        dt = context.dt();

      Vector3 position = ps->particlePosition(i);
      Vector3 motionAwayFromOthers(0, 0, 0);
      for (unsigned int j = 0; j < kNumParticles; ++j) {
        if (j == i)
          continue;

        Vector3 gap = ps->particlePosition(j) - position;
        float distanceSquared = gap.lengthSquared();
        if (distanceSquared < kSeparationSquared)
          motionAwayFromOthers -= gap;
      }

      // Apply the total force to the particle.
      applyForce(ps, i, context, motionAwayFromOthers);
    }
  }


  //
  // Functions
  //

  static Op* MakeOp(Node* node)
  {
    return new VH_Separation(node);
  }


  //
  // Static members
  //

  const Op::Description VH_Separation::desc(kPluginClass, kPluginHelp, MakeOp);

} // namespace vh
