using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Collections.ObjectModel;
using Microsoft.Xna.Framework;
using Henge3D.Collision;

namespace Henge3D.Physics
{
	public class PhysicsManager : GameComponent, IDisposable
	{
		// default setting values
		const float DefaultMaxTimeStep = 1f / 60f;
		const float DefaultLinearLimit = 50f;
		const float DefaultAngularLimit = 20f;
		const float DefaultLinearDamping = 0.9f;
		const float DefaultAngularDamping = 0.25f;
		const float DefaultMinRestitution = 0.5f;
		const float DefaultDeactivationTime = 1f;
		const float DefaultActiveLinearThreshold = 0.25f;
		const float DefaultActiveAngularThreshold = 0.5f;
		const float DefaultLinearErrorTolerance = 0.001f;
		const float DefaultAngularErrorTolerance = 0.01f;
		const float DefaultPenetrationBias = 5f;
		const float DefaultPositionCorrectionFactor = 0.2f;
		const float DefaultSweepThreshold = 0.25f;
		const int DefaultMaxPointsPerContact = 16;
		const int DefaultContactPoolCapacity = 64;
		const int DefaultIslandPoolCapacity = 16;
		const int DefaultVelocityIterations = 20;
		const int DefaultPositionIterations = 5;
		const bool DefaultIsSolverWarmStarted = true;
		const bool DefaultIsContactListSorted = true;

		// engine settings
		private float _maxTimeStep = DefaultMaxTimeStep;
		private float _linearLimit = DefaultLinearLimit;
		private float _angularLimit = DefaultAngularLimit;
		private float _linearDamping = DefaultLinearDamping;
		private float _angularDamping = DefaultAngularDamping;
		private float _minRestitution = DefaultMinRestitution;
		private float _deactivationTime = DefaultDeactivationTime;
		private float _activeLinearThreshold = DefaultActiveLinearThreshold;
		private float _activeAngularThreshold = DefaultActiveAngularThreshold;
		private float _linearErrorTolerance = DefaultLinearErrorTolerance;
		private float _angularErrorTolerance = DefaultAngularErrorTolerance;
		private float _penetrationBias = DefaultPenetrationBias;
		private float _positionCorrectionFactor = DefaultPositionCorrectionFactor;
		private float _sweepThreshold = DefaultSweepThreshold;
		private int _maxPointsPerContact = DefaultMaxPointsPerContact;
		private int _contactPoolCapacity = DefaultContactPoolCapacity;
		private int _islandPoolCapacity = DefaultIslandPoolCapacity;
		private int _velocityIterations = DefaultVelocityIterations;
		private int _positionIterations = DefaultPositionIterations;
		private bool _isSolverWarmStarted = DefaultIsSolverWarmStarted;
		private bool _isContactListSorted = DefaultIsContactListSorted;
		private GravityForce _gravityForce;
		private BroadPhase _broadPhase;

		// runtime state management
		private IAllocator<Island> _islandAlloc;
		private List<RigidBody> _bodies;
		private ReadOnlyCollection<RigidBody> _bodiesReadOnly;
		private List<Island> _islands;
		private List<IForceGenerator> _generators;
		private BodyCollisionFunctor _contacts;
		private ContactCache _contactCache;
		private TaskManager _taskManager;
		private float _dt, _dtInv;

		public PhysicsManager(Game game)
			: base(game)
		{
			_bodies = new List<RigidBody>();
			_bodiesReadOnly = _bodies.AsReadOnly();

			_islands = new List<Island>();
			_generators = new List<IForceGenerator>();
			_gravityForce = new GravityForce(Vector3.Zero);
			game.Components.Add(this);
		}

		// exposed config values

		/// <summary>
		/// Gets or sets the maximum time-step, in seconds, that the physics engine will integrate over. If the current time step is greater
		/// than this value, then objects will appear to slow down.
		/// </summary>
		public float MaxTimeStep { get { return _maxTimeStep; } set { _maxTimeStep = value; } }

		/// <summary>
		/// Gets or sets the maximum allowable linear velocity.
		/// </summary>
		public float LinearLimit { get { return _linearLimit; } set { _linearLimit = value; } }

		/// <summary>
		/// Gets or sets the maximum allowable angular velocity.
		/// </summary>
		public float AngularLimit { get { return _angularLimit; } set { _angularLimit = value; } }

		/// <summary>
		/// Gets or sets the linear damping target, or the amount that objects will be slowed over time. This must be a number between 0
		/// and 1. A lower number causes more damping, a higher number less.
		/// </summary>
		public float LinearDamping { get { return _linearDamping; } set { _linearDamping = value; } }

		/// <summary>
		/// Gets or sets the angular damping target, or the amount that objects will be slowed over time. This must be a number between 0
		/// and 1. A lower number causes more damping, a higher number less.
		/// </summary>
		public float AngularDamping { get { return _angularDamping; } set { _angularDamping = value; } }

		/// <summary>
		/// Gets or sets the minimum amount of restitution that can be applied when two objects collide. Setting this to a non-zero value helps reduce
		/// jitters from "micro-collisions" when two bodies with elastic material are resting against each other.
		/// </summary>
		public float MinRestitution { get { return _minRestitution; } set { _minRestitution = value; } }

		/// <summary>
		/// Gets or sets the amount of time an object must be below the activation threshold to be considered inactive. Objects and groups of objects
		/// that are below the threshold for this period of time will automatically be deactivated.
		/// </summary>
		public float DeactivationTime { get { return _deactivationTime; } set { _deactivationTime = value; } }

		/// <summary>
		/// Gets or sets the minimum linear velocity that an object must maintain to be considered active. If an object's velocity drops below this
		/// threshold for a certain amount of time, it is automatically deactivated.
		/// </summary>
		public float ActiveLinearThreshold { get { return _activeLinearThreshold; } set { _activeLinearThreshold = value; } }

		/// <summary>
		/// Gets or sets the minimum angular velocity that an object must maintain to be considered active. If an object's velocity drops below this
		/// threshold for a certain amount of time, it is automatically deactivated.
		/// </summary>
		public float ActiveAngularThreshold { get { return _activeAngularThreshold; } set { _activeAngularThreshold = value; } }

		/// <summary>
		/// Gets or sets the amount of linear error that constraints will accept without attempting correction. Setting this to a very low value will
		/// cause jittery behavior, and setting it to a high value will result in "loose" constraints.
		/// </summary>
		public float LinearErrorTolerance { get { return _linearErrorTolerance; } set { _linearErrorTolerance = value; } }

		/// <summary>
		/// Gets or sets the amount of angular error that constraints will accept without attempting correction. Setting this to a very low value will
		/// cause jittery behavior, and setting it to a high value will result in "loose" constraints.
		/// </summary>
		public float AngularErrorTolerance { get { return _angularErrorTolerance; } set { _angularErrorTolerance = value; } }

		/// <summary>
		/// Gets or sets the amount of inter-penetration that will be compensated for via velocity changes. Setting this to a high value will result
		/// in "bouncy" collision behavior, and setting it to a low value may result in additional inter-penetration between colliding objects.
		/// </summary>
		public float PenetrationBias { get { return _penetrationBias; } set { _penetrationBias = value; } }

		/// <summary>
		/// Gets or sets the amount of error that is corrected on each iteration of the position solving step in the constraints. Setting this to
		/// a high value may result in instability.
		/// </summary>
		public float PositionCorrectionFactor { get { return _positionCorrectionFactor; } set { _positionCorrectionFactor = value; } }

		/// <summary>
		/// Gets or sets the distance threshold that objects must exceed to be considered "fast-moving." When this threshold is exceeded for a frame,
		/// then swept collisions are used rather than basic overlap collisions.
		/// </summary>
		public float SweepThreshold { get { return _sweepThreshold; } set { _sweepThreshold = value; } }

		/// <summary>
		/// Gets or sets the maximum number of contact points that are processed for each collision.
		/// </summary>
		public int MaxPointsPerContact { get { return _maxPointsPerContact; } set { if (_contacts != null) throw new InvalidOperationException("Property can only be set before initialization."); _maxPointsPerContact = value; } }

		/// <summary>
		/// Gets or sets the default size of the contact pool. Each entry in the contact pool represents a collision between two objects.
		/// </summary>
		public int ContactPoolCapacity { get { return _contactPoolCapacity; } set { if (_contacts != null) throw new InvalidOperationException("Property can only be set before initialization."); _contactPoolCapacity = value; } }

		/// <summary>
		/// Gets or sets the default size of the island pool. Islands are sets of objects that share common constraints, such as a contact constraint
		/// or other constraint added by the game.
		/// </summary>
		public int IslandPoolCapacity { get { return _islandPoolCapacity; } set { if (_islandAlloc != null) throw new InvalidOperationException("Property can only be set before initialization."); _islandPoolCapacity = value; } }

		/// <summary>
		/// Gets or sets the number of velocity solving iterations.
		/// </summary>
		public int VelocityIterations { get { return _velocityIterations; } set { _velocityIterations = value; } }

		/// <summary>
		/// Gets or sets the number of position solving iterations.
		/// </summary>
		public int PositionIterations { get { return _positionIterations; } set { _positionIterations = value; } }

		/// <summary>
		/// Gets or sets a value indicating whether constraints are warm-started. Warm starting uses values from the previous frame to provide
		/// better constraint solutions in the current frame.
		/// </summary>
		public bool IsSolverWarmStarted { get { return _isSolverWarmStarted; } set { _isSolverWarmStarted = value; } }

		/// <summary>
		/// Gets or sets a value determining whether constraints are sorted prior to solving. Setting this to true adds extra overhead each frame.
		/// However, when threading is enabled, contact sorting is required to achieve deterministic behavior.
		/// </summary>
		public bool IsConstraintListSorted { get { return _isContactListSorted; } set { _isContactListSorted = value; } }

		/// <summary>
		/// Gets or sets the amount of gravity applied to all objects.
		/// </summary>
		public Vector3 Gravity { get { return _gravityForce.Gravity; } set { _gravityForce.Gravity = value; } }

		/// <summary>
		/// Gets or sets a reference to the broad phase collision implementation. By default, a SAP (Sweep and Prune) implementation is used.
		/// </summary>
		public BroadPhase BroadPhase { get { return _broadPhase; } set { _broadPhase = value; } }

		// runtime state properties

		/// <summary>
		/// Gets the current time-step in seconds.
		/// </summary>
		public float TimeStep { get { return _dt; } }

		/// <summary>
		/// Gets the collection of bodies currently managed by the physics implementation.
		/// </summary>
		public ReadOnlyCollection<RigidBody> Bodies { get { return _bodiesReadOnly; } }

		// for internal use
		internal ContactCache ContactCache { get { return _contactCache; } }
		internal float ActiveLinearThresholdSquared { get { return _activeLinearThreshold * _activeLinearThreshold; } }
		internal float ActiveAngularThresholdSquared { get { return _activeAngularThreshold * _activeAngularThreshold; } }
		internal float SweepThresholdSquared { get { return _sweepThreshold * _sweepThreshold; } }
		internal float LinearErrorToleranceSquared { get { return _linearErrorTolerance * _linearErrorTolerance; } }
		internal float AngularErrorToleranceSquared { get { return _angularErrorTolerance * _angularErrorTolerance; } }
		internal int GravityAxis { get { return _gravityForce.GravityAxis; } }
		internal float TimeStepInverse { get { return _dtInv; } }

		/// <summary>
		/// Initialize the physics implementation.
		/// </summary>
		public override void Initialize()
		{
			base.Initialize();

			if (_broadPhase == null)
				_broadPhase = new SweepAndPrune();

			_islandAlloc = new Pool<Island>(_islandPoolCapacity, 2);
			_contacts = new BodyCollisionFunctor(this);
			_contactCache = new ContactCache(new ContactCache.Allocator(_contactPoolCapacity, 2, _maxPointsPerContact));
			_taskManager = TaskManager.Current;

			_generators.Add(_gravityForce);
		}

		/// <summary>
		/// Standard XNA Update method. If the IsIntegratedOnUpdate property is set to true, the engine will automatically move forward
		/// one step.
		/// </summary>
		/// <param name="gameTime">The current game time information.</param>
		public override void Update(GameTime gameTime)
		{
			base.Update(gameTime);

			this.Integrate((float)gameTime.ElapsedGameTime.TotalSeconds);
		}

		/// <summary>
		/// Integrates the entire system for a single time step. This method applies forces to objects, performs collision detection, and solves
		/// all constraints. Finally, the object positions are updated.
		/// </summary>
		/// <param name="timeStep">The amount of time, in seconds, to step the simulation forward.</param>
		public void Integrate(float timeStep)
		{
			_dt = Math.Min(timeStep, _maxTimeStep);
			_dtInv = 1f / _dt;

			_islandAlloc.Recycle(_islands);
			_islands.Clear();
			_contacts.Clear();

			// apply gravity and other forces
			for (int i = 0; i < _generators.Count; i++)
			{
				_generators[i].Generate(_bodies);
			}

			// integrate forces and prepare bodies for collision detection
			for (int i = 0; i < _bodies.Count; i++)
			{
				_bodies[i].Contacts.Clear();
				_bodies[i].IntegrateForce(_dt);
			}

			// detect collisions
			_broadPhase.Execute(_contacts);
			_contacts.PropagateContacts();
			_contacts.ProcessSeparations(this.Bodies);

			// organize all objects into islands and solve each island
			BuildIslands();
			ProcessIslands();

			// cache contact data for the next frame
			if (_isSolverWarmStarted)
				_contacts.PopulateCache(_contactCache);
		}

		/// <summary>
		/// Clears all bodies and force generators.
		/// </summary>
		public void Clear()
		{
			for (int i = 0; i < _bodies.Count; i++)
				_bodies[i].Manager = null;
			_bodies.Clear();
			_broadPhase.Clear();
			_generators.Clear();
			_generators.Add(_gravityForce);
		}

		/// <summary>
		/// Add a body to the physics world.
		/// </summary>
		/// <param name="body">The body to add. It must not already be managed by a physics implementation.</param>
		public void Add(RigidBody body)
		{
			if (body.Manager != null)
				throw new ArgumentException("Body is already managed by a physics implementation.");
			_bodies.Add(body);
			_broadPhase.Add(body.Skin);
			body.Manager = this;

			var fg = body as IForceGenerator;
			if (fg != null)
			{
				_generators.Add(fg);
			}
		}

		/// <summary>
		/// Remove a body from the physics world.
		/// </summary>
		/// <param name="body">The body to remove.</param>
		/// <returns>Returns a value indicating whether the body was successfully removed.</returns>
		public bool Remove(RigidBody body)
		{
			var fg = body as IForceGenerator;
			if (fg != null)
			{
				_generators.Remove(fg);
			}

			body.Manager = null;
			_broadPhase.Remove(body.Skin);
			return _bodies.Remove(body);
		}

		/// <summary>
		/// Add a constraint to the physics world. A constraint may affect one or two bodies.
		/// </summary>
		/// <param name="constraint">The constraint to add.</param>
		public void Add(Constraint constraint)
		{
			if (constraint.Manager != null)
				throw new ArgumentException("Constraint is already managed by a physics implementation.");
			if (constraint.BodyA != null) constraint.BodyA.Constraints.Add(constraint);
			if (constraint.BodyB != null) constraint.BodyB.Constraints.Add(constraint);
			constraint.Manager = this;
		}

		/// <summary>
		/// Removes a constraint from the physics world.
		/// </summary>
		/// <param name="constraint">The constraint to remove.</param>
		/// <returns>Returns a value indicating whether the constraint was successfully removed.</returns>
		public bool Remove(Constraint constraint)
		{
			bool removed = false;
			if (constraint.BodyA != null) removed |= constraint.BodyA.Constraints.Remove(constraint);
			if (constraint.BodyB != null) removed |= constraint.BodyB.Constraints.Remove(constraint);
			constraint.Manager = null;
			return removed;
		}

		/// <summary>
		/// Add a force generator to the physics world.
		/// </summary>
		/// <param name="generator">The generator to add.</param>
		public void Add(IForceGenerator generator)
		{
			_generators.Add(generator);
		}

		/// <summary>
		/// Remove a force generator from the physics world.
		/// </summary>
		/// <param name="generator">The generator to remove.</param>
		/// <returns>Returns a value indicating whether the generator was successfully removed.</returns>
		public bool Remove(IForceGenerator generator)
		{
			return _generators.Remove(generator);
		}

		/// <summary>
		/// Add a contraption to the physics world.
		/// </summary>
		/// <param name="contraption">The contraption to add.</param>
		public void Add(Contraption contraption)
		{
			for (int i = 0; i < contraption.Bodies.Count; i++)
				this.Add(contraption.Bodies[i]);
			for (int i = 0; i < contraption.Constraints.Count; i++)
				this.Add(contraption.Constraints[i]);
			for (int i = 0; i < contraption.Generators.Count; i++)
				this.Add(contraption.Constraints[i]);
		}

		/// <summary>
		/// Remove a contraption from the physics world.
		/// </summary>
		/// <param name="contraption">The contraption to remove.</param>
		public void Remove(Contraption contraption)
		{
			for (int i = 0; i < contraption.Bodies.Count; i++)
				this.Remove(contraption.Bodies[i]);
			for (int i = 0; i < contraption.Constraints.Count; i++)
				this.Remove(contraption.Constraints[i]);
			for (int i = 0; i < contraption.Generators.Count; i++)
				this.Remove(contraption.Constraints[i]);
		}

		private void BuildIslands()
		{
			for (int i = 0; i < _bodies.Count; i++)
			{
				var a = _bodies[i];
				if (a.Island == null && a.IsMovable)
				{
					var island = _islandAlloc.Allocate();
					island.Manager = this;
					island.Add(a);
					_islands.Add(island);
				}
			}
		}

		private void ProcessIslands()
		{
			for (int i = 0; i < _islands.Count; i++)
			{
				_taskManager.AddTask(_islands[i].Process);
			}
			_taskManager.Execute();
		}

		protected override void Dispose(bool disposing)
		{
			var idisp = _taskManager as IDisposable;
			if (idisp != null)
				idisp.Dispose();
		}
	}
}
