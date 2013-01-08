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
  static const char* kPluginHelp = "Apply simple rules to particles, to make them behave like a flock.";


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
    void flocking(const ParticleContext& context, ParticleSystem* ps);

    Vector3 centering(const Vector3& position,
                      const Vector3& summedPositions,
                      const int numParticles) const;

    Vector3 separation(const Vector3& position,
                       const ParticleSystem* ps,
                       const unsigned int i,
                       const int numParticles,
                       const float separationSquared) const;

    Vector3 matchVelocity(const Vector3& velocity,
                          const Vector3& summedVelocities,
                          const int numParticles) const;

    Vector3 goalSeeking(const Vector3& position) const;

    Vector3 avoidance(const Vector3& position,
                      const float avoidDistanceSquared) const;

    Vector3 speedLimit(const Vector3& force,
                       const float maxSpeedSquared) const;

  private:
    float _centeringRate;
    float _separation;
    float _velocityMatchRate;
    float _goalAttainRate;
    float _maxSpeed;
    Vector3 _goal;
    Vector3 _avoid;
    bool _avoidEnabled;
    float _avoidDistance;
  };


  //
  // VH_Centering methods
  //

  VH_Centering::VH_Centering(Node* node) :
    ParticleBehaviour(node),
    _centeringRate(100.0f),
    _separation(0.1f),
    _velocityMatchRate(8.0f),
    _goalAttainRate(100.0f),
    _maxSpeed(1.0f),
    _goal(0.0f, 0.0f, 0.0f),
    _avoid(0.0f, 0.0f, 0.0f),
    _avoidEnabled(false),
    _avoidDistance(1.0f)
  {
  }


  void VH_Centering::knobs(Knob_Callback f)
  {
    ParticleBehaviour::knobs(f);
    Float_knob(f, &_centeringRate, "centering_rate", "centering rate");
    Float_knob(f, &_separation, "separation_distance", "separation distance");
    Float_knob(f, &_velocityMatchRate, "velocity_match_rate", "velocity match rate");
    Float_knob(f, &_goalAttainRate, "goal_attainment_rate", "goal attainment rate");
    Float_knob(f, &_maxSpeed, "maximum_speed", "maximum speed");
    XYZ_knob(f, &_goal.x, "goal");

    XYZ_knob(f, &_avoid.x, "avoid");
    Bool_knob(f, &_avoidEnabled, "avoid_enabled", "enable");
    Float_knob(f, &_avoidDistance, "avoid_distance", "avoid distance");
  }


#if NUKE_VERSION_MAJOR >= 7
  bool VH_Centering::applyBehaviour(const ParticleContext& context, ParticleSystem* ps)
  {
    flocking(context, ps);
    return true;
  }
#else
  void VH_Centering::applyBehaviour(const ParticleContext& context, ParticleSystem* ps)
  {
    flocking(context, ps);
  }
#endif


  void VH_Centering::flocking(const ParticleContext& context, ParticleSystem* ps)
  {
    const unsigned int kNumParticles = ps->numParticles();
    const float kSeparationSquared = _separation * _separation;
    const float kMaxSpeedSquared = _maxSpeed * _maxSpeed;
    const float kAvoidDistanceSquared = _avoidDistance * _avoidDistance;
    const double kEnd = context.endTime();

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
      Vector3 force = -velocity;

      // Rule 1: particles try to move towards the centre of the flock.
      force += centering(position, summedPositions, kNumParticles);

      // Rule 2: particles try to maintain a minimum separation from each other.
      force += separation(position, ps, i, kNumParticles, kSeparationSquared);

      // Rule 3: particles try to match velocity with each other.
      force += matchVelocity(velocity, summedVelocities, kNumParticles);

      // Rule 4: particles try to move towards a common goal.
      force += goalSeeking(position);

      // Rule 5: avoid a specified location.
      if (_avoidEnabled)
        force += avoidance(position, kAvoidDistanceSquared);

      // Rule 6: limit the maximum speed of movement.
      force = speedLimit(force, kMaxSpeedSquared);

      // Apply the total force to the particle.
      applyForce(ps, i, context, force);
    }
  }


  Vector3 VH_Centering::centering(const Vector3& position,
                                 const Vector3& summedPositions,
                                 const int numParticles) const
  {
    Vector3 perceivedCenter = (summedPositions - position) / (numParticles - 1);
    Vector3 motionTowardsCenter = (perceivedCenter - position) / _centeringRate;
    return motionTowardsCenter;
  }


  Vector3 VH_Centering::separation(const Vector3& position,
                                  const ParticleSystem* ps,
                                  const unsigned int i,
                                  const int numParticles,
                                  const float separationSquared) const
  {
    Vector3 motionAwayFromOthers(0, 0, 0);
    for (unsigned int j = 0; j < numParticles; ++j) {
      if (j == i)
        continue;

      Vector3 gap = ps->particlePosition(j) - position;
      float distanceSquared = gap.lengthSquared();
      if (distanceSquared < separationSquared)
        motionAwayFromOthers -= gap;
    }
    return motionAwayFromOthers;
  }


  Vector3 VH_Centering::matchVelocity(const Vector3& velocity,
                                     const Vector3& summedVelocities,
                                     const int numParticles) const
  {
    Vector3 perceivedVelocity =
      (summedVelocities - velocity) / (numParticles - 1);
    Vector3 motionWithFlock =
      (perceivedVelocity - velocity) / _velocityMatchRate;
    return motionWithFlock;
  }


  Vector3 VH_Centering::goalSeeking(const Vector3& position) const
  {
    Vector3 motionTowardsGoal = (_goal - position) / _goalAttainRate;
    return motionTowardsGoal;
  }


  Vector3 VH_Centering::avoidance(const Vector3& position,
                                 const float avoidDistanceSquared) const
  {
    Vector3 gap = position - _avoid;
    float distanceFromAvoidSquared = gap.lengthSquared();
    if (distanceFromAvoidSquared < avoidDistanceSquared) {
      float distanceFromAvoid = sqrtf(distanceFromAvoidSquared);
      Vector3 normal = gap / distanceFromAvoid;

      float distanceInside = _avoidDistance - distanceFromAvoid;
      return normal * distanceInside;
    }
    else {
      return Vector3(0, 0, 0);
    }
  }


  Vector3 VH_Centering::speedLimit(const Vector3& force,
                                  const float maxSpeedSquared) const
  {
    float speedSquared = force.lengthSquared();
    if (speedSquared > maxSpeedSquared) {
      float speed = sqrtf(speedSquared);
      Vector3 newForce = force / speed * _maxSpeed;
      return newForce;
    }
    else {
      return force;
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

