#include <DDImage/Knob.h>
#include <DDImage/Knobs.h>
#include <DDImage/ParticleOp.h>
#include <DDImage/Vector3.h>

using namespace DD::Image;

namespace vh {

  //
  // Constants
  //

  static const char* kPluginClass = "VH_Flocking";
  static const char* kPluginHelp = "Apply simple rules to particles, to make them behave like a flock.";


  //
  // VH_Flocking class
  //

  class VH_Flocking : public ParticleBehaviour {
  public:
    VH_Flocking(Node* node) :
      ParticleBehaviour(node),
      _centeringRate(100.0f),
      _separation(0.1f),
      _velocityMatchRate(8.0f),
      _goalAttainRate(100.0f),
      _maxSpeed(1.0f),
      _goal(0.0f, 0.0f, 0.0f)
    {}
    virtual ~VH_Flocking() {}

    virtual const char* Class() const { return kPluginClass; }
    virtual const char* node_help() const { return kPluginHelp; }

    virtual void knobs(Knob_Callback f);
    virtual void applyBehaviour(const ParticleContext& context, ParticleSystem* ps);

    static const Description desc;

  private:
    float _centeringRate;
    float _separation;
    float _velocityMatchRate;
    float _goalAttainRate;
    float _maxSpeed;
    Vector3 _goal;
  };


  //
  // VH_Flocking methods
  //

  void VH_Flocking::knobs(Knob_Callback f)
  {
    ParticleBehaviour::knobs(f);
    Float_knob(f, &_centeringRate, "centering rate");
    Float_knob(f, &_separation, "separation distance");
    Float_knob(f, &_velocityMatchRate, "velocity match rate");
    Float_knob(f, &_goalAttainRate, "goal attainment rate");
    Float_knob(f, &_maxSpeed, "maximum speed");
    XYZ_knob(f, &_goal.x, "goal");
  }


  void VH_Flocking::applyBehaviour(const ParticleContext& context, ParticleSystem* ps)
  {
    const unsigned int kNumParticles = ps->numParticles();
    const float kSeparationSquared = _separation * _separation;
    const float kMaxSpeedSquared = _maxSpeed * _maxSpeed;
    const double kEnd = context.endTime();

    // XXX doesn't handle subframe stepping yet.

    // Pre-calculations
    Vector3 summedPositions(0, 0, 0);
    Vector3 summedVelocities(0, 0, 0);
    for (unsigned int i = 0; i < kNumParticles; ++i) {
      summedPositions += ps->particlePosition(i);
      summedVelocities += ps->particleVelocity(i);
    }

    // Apply the flocking rules to each particle.
    for (unsigned int i = 0; i < kNumParticles; ++i) {
      double startTime = ps->particleStartTime(i);
      if (startTime > kEnd)
        continue;

      double dt = kEnd - startTime;
      if (dt > context.dt())
        dt = context.dt();

      Vector3 position = ps->particlePosition(i);
      Vector3 velocity = ps->particleVelocity(i);

      Vector3 force(0, 0, 0);

      // Rule 1: particles try to move towards the centre of the flock.
      Vector3 perceivedCenter =
          (summedPositions - position) / (kNumParticles - 1);
      Vector3 motionTowardsCenter =
        (perceivedCenter - position) / _centeringRate;
      force += motionTowardsCenter;

      // Rule 2: particles try to maintain a minimum separation from each other.
      Vector3 motionAwayFromOthers(0, 0, 0);
      for (unsigned int j = 0; j < kNumParticles; ++j) {
        if (j == i)
          continue;

        Vector3 gap = (ps->particlePosition(j) - position);
        float distanceSquared = gap.lengthSquared();
        if (distanceSquared < kSeparationSquared)
          motionAwayFromOthers -= gap;
      }
      force += motionAwayFromOthers;

      // Rule 3: particles try to match velocity with each other.
      Vector3 perceivedVelocity =
        (summedVelocities - velocity) / (kNumParticles - 1);
      Vector3 motionWithFlock =
        (perceivedVelocity - velocity) / _velocityMatchRate;
      force += motionWithFlock;

      // Rule 4: particles try to move towards a common goal.
      Vector3 motionTowardsGoal = (_goal - position) / _goalAttainRate;
      force += motionTowardsGoal;

      // Rule 5: limit the maximum speed of movement.
      float speedSquared = force.lengthSquared();
      if (speedSquared > kMaxSpeedSquared) {
        float speed = sqrtf(speedSquared);
        force = force / speed * _maxSpeed;
      }

      // Apply the total force to the particle.
      ps->particlePosition(i) += force * dt;
      //applyForce(ps, i, context, force);
    }
  }


  //
  // Functions
  //

  static Op* MakeOp(Node* node)
  {
    return new VH_Flocking(node);
  }


  //
  // Static members
  //

  const Op::Description VH_Flocking::desc(kPluginClass, kPluginHelp, MakeOp);

} // namespace vh

