"""
Trossen Arm Python Bindings
"""
from __future__ import annotations
import collections.abc
import numpy
import numpy.typing
import typing
__all__: list[str] = ['AlgorithmParameter', 'ArrayDouble3', 'ArrayDouble6', 'ArrayDouble9', 'EndEffector', 'IPMethod', 'InterpolationSpace', 'JointCharacteristic', 'JointLimit', 'Link', 'LogicError', 'Mode', 'Model', 'MotorParameter', 'PIDParameter', 'RobotOutput', 'RuntimeError', 'StandardEndEffector', 'TrossenArmDriver', 'VectorDouble']
class AlgorithmParameter:
    """
    Algorithm parameter
    """
    def __init__(self) -> None:
        ...
    @property
    def singularity_threshold(self) -> float:
        """
                Threshold for singularity detection
        """
    @singularity_threshold.setter
    def singularity_threshold(self, arg0: typing.SupportsFloat) -> None:
        ...
class ArrayDouble3:
    __hash__: typing.ClassVar[None] = None
    def __bool__(self) -> bool:
        """
        Check whether the array is nonempty
        """
    def __contains__(self, x: typing.SupportsFloat) -> bool:
        """
        Return true the container contains ``x``
        """
    def __eq__(self, arg0: ArrayDouble3) -> bool:
        ...
    @typing.overload
    def __getitem__(self, s: slice) -> numpy.typing.NDArray[numpy.float64]:
        """
        Retrieve list elements using a slice object
        """
    @typing.overload
    def __getitem__(self, arg0: typing.SupportsInt) -> float:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: ArrayDouble3) -> None:
        """
        Copy constructor
        """
    @typing.overload
    def __init__(self, arg0: collections.abc.Iterable) -> None:
        ...
    def __iter__(self) -> collections.abc.Iterator[float]:
        ...
    def __len__(self) -> int:
        ...
    def __ne__(self, arg0: ArrayDouble3) -> bool:
        ...
    def __repr__(self) -> str:
        """
        Return the canonical string representation of this list.
        """
    @typing.overload
    def __setitem__(self, arg0: typing.SupportsInt, arg1: typing.SupportsFloat) -> None:
        ...
    @typing.overload
    def __setitem__(self, arg0: slice, arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float64]) -> None:
        """
        Assign list elements using a slice object
        """
    def count(self, x: typing.SupportsFloat) -> int:
        """
        Return the number of times ``x`` appears in the list
        """
class ArrayDouble6:
    __hash__: typing.ClassVar[None] = None
    def __bool__(self) -> bool:
        """
        Check whether the array is nonempty
        """
    def __contains__(self, x: typing.SupportsFloat) -> bool:
        """
        Return true the container contains ``x``
        """
    def __eq__(self, arg0: ArrayDouble6) -> bool:
        ...
    @typing.overload
    def __getitem__(self, s: slice) -> numpy.typing.NDArray[numpy.float64]:
        """
        Retrieve list elements using a slice object
        """
    @typing.overload
    def __getitem__(self, arg0: typing.SupportsInt) -> float:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: ArrayDouble6) -> None:
        """
        Copy constructor
        """
    @typing.overload
    def __init__(self, arg0: collections.abc.Iterable) -> None:
        ...
    def __iter__(self) -> collections.abc.Iterator[float]:
        ...
    def __len__(self) -> int:
        ...
    def __ne__(self, arg0: ArrayDouble6) -> bool:
        ...
    def __repr__(self) -> str:
        """
        Return the canonical string representation of this list.
        """
    @typing.overload
    def __setitem__(self, arg0: typing.SupportsInt, arg1: typing.SupportsFloat) -> None:
        ...
    @typing.overload
    def __setitem__(self, arg0: slice, arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float64]) -> None:
        """
        Assign list elements using a slice object
        """
    def count(self, x: typing.SupportsFloat) -> int:
        """
        Return the number of times ``x`` appears in the list
        """
class ArrayDouble9:
    __hash__: typing.ClassVar[None] = None
    def __bool__(self) -> bool:
        """
        Check whether the array is nonempty
        """
    def __contains__(self, x: typing.SupportsFloat) -> bool:
        """
        Return true the container contains ``x``
        """
    def __eq__(self, arg0: ArrayDouble9) -> bool:
        ...
    @typing.overload
    def __getitem__(self, s: slice) -> numpy.typing.NDArray[numpy.float64]:
        """
        Retrieve list elements using a slice object
        """
    @typing.overload
    def __getitem__(self, arg0: typing.SupportsInt) -> float:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: ArrayDouble9) -> None:
        """
        Copy constructor
        """
    @typing.overload
    def __init__(self, arg0: collections.abc.Iterable) -> None:
        ...
    def __iter__(self) -> collections.abc.Iterator[float]:
        ...
    def __len__(self) -> int:
        ...
    def __ne__(self, arg0: ArrayDouble9) -> bool:
        ...
    def __repr__(self) -> str:
        """
        Return the canonical string representation of this list.
        """
    @typing.overload
    def __setitem__(self, arg0: typing.SupportsInt, arg1: typing.SupportsFloat) -> None:
        ...
    @typing.overload
    def __setitem__(self, arg0: slice, arg1: typing.Annotated[numpy.typing.ArrayLike, numpy.float64]) -> None:
        """
        Assign list elements using a slice object
        """
    def count(self, x: typing.SupportsFloat) -> int:
        """
        Return the number of times ``x`` appears in the list
        """
class EndEffector:
    """
    End effector properties
    """
    def __init__(self) -> None:
        ...
    @property
    def finger_left(self) -> Link:
        """
                Properties of the left finger link
        """
    @finger_left.setter
    def finger_left(self, arg0: Link) -> None:
        ...
    @property
    def finger_right(self) -> Link:
        """
                Properties of the right finger link
        """
    @finger_right.setter
    def finger_right(self, arg0: Link) -> None:
        ...
    @property
    def offset_finger_left(self) -> float:
        """
                Offset from the palm center to the left carriage center in m in home configuration
        """
    @offset_finger_left.setter
    def offset_finger_left(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def offset_finger_right(self) -> float:
        """
                Offset from the palm center to the right carriage center in m in home configuration
        """
    @offset_finger_right.setter
    def offset_finger_right(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def palm(self) -> Link:
        """
                Properties of the palm link
        """
    @palm.setter
    def palm(self, arg0: Link) -> None:
        ...
    @property
    def pitch_circle_radius(self) -> float:
        """
                Pitch circle radius in m
        """
    @pitch_circle_radius.setter
    def pitch_circle_radius(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def t_flange_tool(self) -> ArrayDouble6:
        """
                The tool frame pose measured in the flange frame
        
                Notes
                -----
                The first 3 elements are the translation and the last 3 elements are the
                angle-axis representation of the rotation
        """
    @t_flange_tool.setter
    def t_flange_tool(self, arg0: ArrayDouble6) -> None:
        ...
class IPMethod:
    """
    @brief IP methods
    
    Members:
    
      manual : Use the manual IP address specified in the configuration
    
      dhcp : Use the DHCP to obtain the IP address, if failed, use the default IP address
    """
    __members__: typing.ClassVar[dict[str, IPMethod]]  # value = {'manual': <IPMethod.manual: 0>, 'dhcp': <IPMethod.dhcp: 1>}
    dhcp: typing.ClassVar[IPMethod]  # value = <IPMethod.dhcp: 1>
    manual: typing.ClassVar[IPMethod]  # value = <IPMethod.manual: 0>
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class InterpolationSpace:
    """
    @brief Interpolation spaces
    
    Members:
    
      joint : Joint space
    
      cartesian : Cartesian space
    """
    __members__: typing.ClassVar[dict[str, InterpolationSpace]]  # value = {'joint': <InterpolationSpace.joint: 0>, 'cartesian': <InterpolationSpace.cartesian: 1>}
    cartesian: typing.ClassVar[InterpolationSpace]  # value = <InterpolationSpace.cartesian: 1>
    joint: typing.ClassVar[InterpolationSpace]  # value = <InterpolationSpace.joint: 0>
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class JointCharacteristic:
    """
    Joint characteristic
    """
    def __init__(self) -> None:
        ...
    @property
    def effort_correction(self) -> float:
        """
                Effort correction in motor effort unit / Nm or N
        
                Notes
                -----
                It must be within [0.2, 5.0]
        """
    @effort_correction.setter
    def effort_correction(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def friction_constant_term(self) -> float:
        """
                Friction constant term in Nm for arm joints or N for the gripper joint
        """
    @friction_constant_term.setter
    def friction_constant_term(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def friction_coulomb_coef(self) -> float:
        """
                Friction coulomb coef in Nm/Nm for arm joints or N/N for the gripper joint
        """
    @friction_coulomb_coef.setter
    def friction_coulomb_coef(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def friction_transition_velocity(self) -> float:
        """
                Friction transition velocity in rad/s for arm joints or m/s for the gripper joint
        
                Notes
                -----
                It must be positive
        """
    @friction_transition_velocity.setter
    def friction_transition_velocity(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def friction_viscous_coef(self) -> float:
        """
                Friction viscous coef in Nm/(rad/s) for arm joints or N/(m/s) for the gripper joint
        """
    @friction_viscous_coef.setter
    def friction_viscous_coef(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def position_offset(self) -> float:
        """
                Position offset in rad for arm joints or m for the gripper joint
        """
    @position_offset.setter
    def position_offset(self, arg0: typing.SupportsFloat) -> None:
        ...
class JointLimit:
    """
    Joint limits
    """
    def __init__(self) -> None:
        ...
    @property
    def effort_max(self) -> float:
        """
                Maximum effort in Nm for arm joints and N for gripper
        """
    @effort_max.setter
    def effort_max(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def effort_tolerance(self) -> float:
        """
                Tolerance on output effort in Nm for arm joints and N for gripper
        """
    @effort_tolerance.setter
    def effort_tolerance(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def position_max(self) -> float:
        """
                Maximum position in rad for arm joints and m for gripper
        """
    @position_max.setter
    def position_max(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def position_min(self) -> float:
        """
                Minimum position in rad for arm joints and m for gripper
        """
    @position_min.setter
    def position_min(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def position_tolerance(self) -> float:
        """
                Tolerance on output position in rad for arm joints and m for gripper
        """
    @position_tolerance.setter
    def position_tolerance(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def velocity_max(self) -> float:
        """
                Maximum velocity in rad/s for arm joints and m/s for gripper
        """
    @velocity_max.setter
    def velocity_max(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def velocity_tolerance(self) -> float:
        """
                Tolerance on output velocity in rad/s for arm joints and m/s for gripper
        """
    @velocity_tolerance.setter
    def velocity_tolerance(self, arg0: typing.SupportsFloat) -> None:
        ...
class Link:
    """
    Link properties
    """
    def __init__(self) -> None:
        ...
    @property
    def inertia(self) -> ArrayDouble9:
        """
                Inertia in kg m^2
        """
    @inertia.setter
    def inertia(self, arg0: ArrayDouble9) -> None:
        ...
    @property
    def mass(self) -> float:
        """
                Mass in kg
        """
    @mass.setter
    def mass(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def origin_rpy(self) -> ArrayDouble3:
        """
                Inertia frame RPY angles measured in link frame in rad
        """
    @origin_rpy.setter
    def origin_rpy(self, arg0: ArrayDouble3) -> None:
        ...
    @property
    def origin_xyz(self) -> ArrayDouble3:
        """
                Inertia frame translation measured in link frame in m
        """
    @origin_xyz.setter
    def origin_xyz(self, arg0: ArrayDouble3) -> None:
        ...
class LogicError(AssertionError):
    pass
class Mode:
    """
    Operation modes of a joint
    
    Members:
    
      idle : All joints are braked
    
      position : Control the joint to a desired position
    
      velocity : Control the joint to a desired velocity
    
      external_effort : Control the joint to a desired external effort
    
      effort : Control the joint to a desired effort
    """
    __members__: typing.ClassVar[dict[str, Mode]]  # value = {'idle': <Mode.idle: 0>, 'position': <Mode.position: 1>, 'velocity': <Mode.velocity: 2>, 'external_effort': <Mode.external_effort: 3>, 'effort': <Mode.effort: 4>}
    effort: typing.ClassVar[Mode]  # value = <Mode.effort: 4>
    external_effort: typing.ClassVar[Mode]  # value = <Mode.external_effort: 3>
    idle: typing.ClassVar[Mode]  # value = <Mode.idle: 0>
    position: typing.ClassVar[Mode]  # value = <Mode.position: 1>
    velocity: typing.ClassVar[Mode]  # value = <Mode.velocity: 2>
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class Model:
    """
    @brief Robot models
    
    Members:
    
      wxai_v0 : WXAI V0
    
      vxai_v0_right : VXAI V0 RIGHT
    
      vxai_v0_left : VXAI V0 LEFT
    """
    __members__: typing.ClassVar[dict[str, Model]]  # value = {'wxai_v0': <Model.wxai_v0: 0>, 'vxai_v0_right': <Model.vxai_v0_right: 1>, 'vxai_v0_left': <Model.vxai_v0_left: 2>}
    vxai_v0_left: typing.ClassVar[Model]  # value = <Model.vxai_v0_left: 2>
    vxai_v0_right: typing.ClassVar[Model]  # value = <Model.vxai_v0_right: 1>
    wxai_v0: typing.ClassVar[Model]  # value = <Model.wxai_v0: 0>
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class MotorParameter:
    """
    Motor parameters
    """
    def __init__(self) -> None:
        ...
    @property
    def position(self) -> PIDParameter:
        """
        Position loop PID parameters
        """
    @position.setter
    def position(self, arg0: PIDParameter) -> None:
        ...
    @property
    def velocity(self) -> PIDParameter:
        """
        Velocity loop PID parameters
        """
    @velocity.setter
    def velocity(self, arg0: PIDParameter) -> None:
        ...
class PIDParameter:
    """
    PID parameters
    """
    def __init__(self) -> None:
        ...
    @property
    def imax(self) -> float:
        """
        Maximum integral value
        """
    @imax.setter
    def imax(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def kd(self) -> float:
        """
        Derivative gain
        """
    @kd.setter
    def kd(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def ki(self) -> float:
        """
        Integral gain
        """
    @ki.setter
    def ki(self, arg0: typing.SupportsFloat) -> None:
        ...
    @property
    def kp(self) -> float:
        """
        Proportional gain
        """
    @kp.setter
    def kp(self, arg0: typing.SupportsFloat) -> None:
        ...
class RobotOutput:
    """
    Robot output
    """
    class Cartesian:
        """
        Cartesian outputs
        """
        def __init__(self) -> None:
            ...
        @property
        def accelerations(self) -> ArrayDouble6:
            """
                    Spatial acceleration of the end effector frame with respect to the base frame measured in
                    the base frame in m/s^2 and rad/s^2
            
                    Notes
                    -----
                    The first 3 elements are the linear acceleration and the last 3 elements are the angular
                    acceleration
            """
        @accelerations.setter
        def accelerations(self, arg0: ArrayDouble6) -> None:
            ...
        @property
        def external_efforts(self) -> ArrayDouble6:
            """
                    Spatial external efforts applied to the end effector frame measured in the base frame in
                    N and Nm
            
                    Notes
                    -----
                    The first 3 elements are the force and the last 3 elements are the torque
            
                    All external efforts are assumed to be applied at the end effector frame
            """
        @external_efforts.setter
        def external_efforts(self, arg0: ArrayDouble6) -> None:
            ...
        @property
        def positions(self) -> ArrayDouble6:
            """
                    Spatial position of the end effector frame measured in the base frame in m and rad
            
                    Notes
                    -----
                    The first 3 elements are the translation and the last 3 elements are the angle-axis
                    representation of the rotation
            """
        @positions.setter
        def positions(self, arg0: ArrayDouble6) -> None:
            ...
        @property
        def velocities(self) -> ArrayDouble6:
            """
                    Spatial velocity of the end effector frame with respect to the base frame measured in the
                    base frame in m/s and rad/s
            
                    Notes
                    -----
                    The first 3 elements are the linear velocity and the last 3 elements are the angular
                    velocity
            """
        @velocities.setter
        def velocities(self, arg0: ArrayDouble6) -> None:
            ...
    class Header:
        """
        Header
        """
        def __init__(self) -> None:
            ...
        @property
        def id(self) -> int:
            """
                    Consecutively increasing ID since configuration
            """
        @id.setter
        def id(self, arg0: typing.SupportsInt) -> None:
            ...
        @property
        def timestamp(self) -> int:
            """
                    Timestamp in microseconds since configuration
            """
        @timestamp.setter
        def timestamp(self, arg0: typing.SupportsInt) -> None:
            ...
    class Joint:
        """
        Outputs in joint space
        """
        class All:
            """
            Outputs of all joints
            """
            def __init__(self) -> None:
                ...
            @property
            def accelerations(self) -> VectorDouble:
                """
                        Accelerations in rad/s^2 for arm joints and m/s^2 for the gripper joint
                """
            @accelerations.setter
            def accelerations(self, arg0: VectorDouble) -> None:
                ...
            @property
            def compensation_efforts(self) -> VectorDouble:
                """
                        Compensation efforts in Nm for arm joints and N for the gripper joint
                """
            @compensation_efforts.setter
            def compensation_efforts(self, arg0: VectorDouble) -> None:
                ...
            @property
            def driver_temperatures(self) -> VectorDouble:
                """
                        Driver temperatures in C
                """
            @driver_temperatures.setter
            def driver_temperatures(self, arg0: VectorDouble) -> None:
                ...
            @property
            def efforts(self) -> VectorDouble:
                """
                        Efforts in Nm for arm joints and N for the gripper joint
                """
            @efforts.setter
            def efforts(self, arg0: VectorDouble) -> None:
                ...
            @property
            def external_efforts(self) -> VectorDouble:
                """
                        External efforts in Nm for arm joints and N for the gripper joint
                """
            @external_efforts.setter
            def external_efforts(self, arg0: VectorDouble) -> None:
                ...
            @property
            def positions(self) -> VectorDouble:
                """
                        Positions in rad for arm joints and m for the gripper joint
                """
            @positions.setter
            def positions(self, arg0: VectorDouble) -> None:
                ...
            @property
            def rotor_temperatures(self) -> VectorDouble:
                """
                        Rotor temperatures in C
                """
            @rotor_temperatures.setter
            def rotor_temperatures(self, arg0: VectorDouble) -> None:
                ...
            @property
            def velocities(self) -> VectorDouble:
                """
                        Velocities in rad/s for arm joints and m/s for the gripper joint
                """
            @velocities.setter
            def velocities(self, arg0: VectorDouble) -> None:
                ...
        class Arm:
            """
            Outputs of the arm joints
            """
            def __init__(self) -> None:
                ...
            @property
            def accelerations(self) -> VectorDouble:
                """
                        Accelerations in rad/s^2
                """
            @accelerations.setter
            def accelerations(self, arg0: VectorDouble) -> None:
                ...
            @property
            def compensation_efforts(self) -> VectorDouble:
                """
                        Compensation efforts in Nm
                """
            @compensation_efforts.setter
            def compensation_efforts(self, arg0: VectorDouble) -> None:
                ...
            @property
            def driver_temperatures(self) -> VectorDouble:
                """
                        Driver temperatures in C
                """
            @driver_temperatures.setter
            def driver_temperatures(self, arg0: VectorDouble) -> None:
                ...
            @property
            def efforts(self) -> VectorDouble:
                """
                        Efforts in Nm
                """
            @efforts.setter
            def efforts(self, arg0: VectorDouble) -> None:
                ...
            @property
            def external_efforts(self) -> VectorDouble:
                """
                        External efforts in Nm
                """
            @external_efforts.setter
            def external_efforts(self, arg0: VectorDouble) -> None:
                ...
            @property
            def positions(self) -> VectorDouble:
                """
                        Positions in rad
                """
            @positions.setter
            def positions(self, arg0: VectorDouble) -> None:
                ...
            @property
            def rotor_temperatures(self) -> VectorDouble:
                """
                        Rotor temperatures in C
                """
            @rotor_temperatures.setter
            def rotor_temperatures(self, arg0: VectorDouble) -> None:
                ...
            @property
            def velocities(self) -> VectorDouble:
                """
                        Velocities in rad/s
                """
            @velocities.setter
            def velocities(self, arg0: VectorDouble) -> None:
                ...
        class Gripper:
            """
            Outputs of the gripper joint
            """
            def __init__(self) -> None:
                ...
            @property
            def acceleration(self) -> float:
                """
                        Acceleration in m/s^2
                """
            @acceleration.setter
            def acceleration(self, arg0: typing.SupportsFloat) -> None:
                ...
            @property
            def compensation_effort(self) -> float:
                """
                        Compensation effort in N
                """
            @compensation_effort.setter
            def compensation_effort(self, arg0: typing.SupportsFloat) -> None:
                ...
            @property
            def driver_temperature(self) -> float:
                """
                        Driver temperature in C
                """
            @driver_temperature.setter
            def driver_temperature(self, arg0: typing.SupportsFloat) -> None:
                ...
            @property
            def effort(self) -> float:
                """
                        Effort in N
                """
            @effort.setter
            def effort(self, arg0: typing.SupportsFloat) -> None:
                ...
            @property
            def external_effort(self) -> float:
                """
                        External effort in N
                """
            @external_effort.setter
            def external_effort(self, arg0: typing.SupportsFloat) -> None:
                ...
            @property
            def position(self) -> float:
                """
                        Position in m
                """
            @position.setter
            def position(self, arg0: typing.SupportsFloat) -> None:
                ...
            @property
            def rotor_temperature(self) -> float:
                """
                        Rotor temperature in C
                """
            @rotor_temperature.setter
            def rotor_temperature(self, arg0: typing.SupportsFloat) -> None:
                ...
            @property
            def velocity(self) -> float:
                """
                        Velocity in m/s
                """
            @velocity.setter
            def velocity(self, arg0: typing.SupportsFloat) -> None:
                ...
        def __init__(self) -> None:
            ...
        @property
        def all(self) -> RobotOutput.Joint.All:
            """
            Outputs of all joints
            """
        @all.setter
        def all(self, arg0: RobotOutput.Joint.All) -> None:
            ...
        @property
        def arm(self) -> RobotOutput.Joint.Arm:
            """
            Outputs of the arm joints
            """
        @arm.setter
        def arm(self, arg0: RobotOutput.Joint.Arm) -> None:
            ...
        @property
        def gripper(self) -> RobotOutput.Joint.Gripper:
            """
            Outputs of the gripper joint
            """
        @gripper.setter
        def gripper(self, arg0: RobotOutput.Joint.Gripper) -> None:
            ...
    def __init__(self) -> None:
        ...
    @property
    def cartesian(self) -> RobotOutput.Cartesian:
        """
        Outputs in Cartesian space
        """
    @cartesian.setter
    def cartesian(self, arg0: RobotOutput.Cartesian) -> None:
        ...
    @property
    def header(self) -> RobotOutput.Header:
        """
        Header
        """
    @header.setter
    def header(self, arg0: RobotOutput.Header) -> None:
        ...
    @property
    def joint(self) -> RobotOutput.Joint:
        """
        Outputs in joint space
        """
    @joint.setter
    def joint(self, arg0: RobotOutput.Joint) -> None:
        ...
class RuntimeError(RuntimeError):
    pass
class StandardEndEffector:
    """
    End effector properties for the standard variants
    """
    no_gripper: typing.ClassVar[EndEffector]  # value = <trossen_arm.EndEffector object>
    vxai_v0_base: typing.ClassVar[EndEffector]  # value = <trossen_arm.EndEffector object>
    wxai_v0_base: typing.ClassVar[EndEffector]  # value = <trossen_arm.EndEffector object>
    wxai_v0_follower: typing.ClassVar[EndEffector]  # value = <trossen_arm.EndEffector object>
    wxai_v0_leader: typing.ClassVar[EndEffector]  # value = <trossen_arm.EndEffector object>
    def __init__(self) -> None:
        ...
class TrossenArmDriver:
    """
    Trossen Arm Driver
    """
    @staticmethod
    def get_default_logger_name() -> str:
        """
                Get the default logger name.
        
                Returns
                -------
                str
                    Default logger name.
        """
    @staticmethod
    def get_logger_name(arg0: Model, arg1: str) -> str:
        """
                Get the logger name.
        
                Parameters
                ----------
                model : Model
                    Model of the robot
                serv_ip : str
                    IP address of the robot
        
                Returns
                -------
                str
                    Logger name.
        """
    def __init__(self) -> None:
        ...
    def cleanup(self, reboot_controller: bool = False) -> None:
        """
                Cleanup the driver.
        
                Parameters
                ----------
                reboot_controller : bool, optional
                    Whether to reboot the controller, default is false.
        """
    def configure(self, model: Model, end_effector: EndEffector, serv_ip: str, clear_error: bool, timeout: typing.SupportsFloat = 20.0) -> None:
        """
                Configure the driver.
        
                Parameters
                ----------
                model : Model
                    Model of the robot.
                end_effector : EndEffector
                    End effector properties.
                serv_ip : str
                    IP address of the robot.
                clear_error : bool
                    Whether to clear the error state of the robot.
                timeout : float, optional
                    Timeout for connection to the arm controller's TCP server in seconds, default is 20.0s.
        """
    def get_algorithm_parameter(self) -> AlgorithmParameter:
        """
                Get the algorithm parameter.
        
                Returns
                -------
                AlgorithmParameter
                    Parameter used for robotic algorithms.
        """
    def get_all_accelerations(self) -> VectorDouble:
        """
                Get the accelerations of all joints.
        
                Returns
                -------
                list of float
                    Accelerations in rad/s^2 for arm joints and m/s^2 for the gripper joint.
        """
    def get_all_compensation_efforts(self) -> VectorDouble:
        """
                Get the compensation efforts of all joints.
        
                Returns
                -------
                list of float
                    Compensation efforts in Nm for arm joints and N for the gripper joint.
        """
    def get_all_driver_temperatures(self) -> VectorDouble:
        """
                Get the driver temperatures of all joints.
        
                Returns
                -------
                list of float
                    Driver temperatures in C.
        """
    def get_all_efforts(self) -> VectorDouble:
        """
                Get the efforts of all joints.
        
                Returns
                -------
                list of float
                    Efforts in Nm for arm joints and N for the gripper joint.
        """
    def get_all_external_efforts(self) -> VectorDouble:
        """
                Get the external efforts of all joints.
        
                Returns
                -------
                list of float
                    External efforts in Nm for arm joints and N for the gripper joint.
        """
    def get_all_positions(self) -> VectorDouble:
        """
                Get the positions of all joints.
        
                Returns
                -------
                list of float
                    Positions in rad for arm joints and m for the gripper joint.
        """
    def get_all_rotor_temperatures(self) -> VectorDouble:
        """
                Get the rotor temperatures of all joints.
        
                Returns
                -------
                list of float
                    Rotor temperatures in C.
        """
    def get_all_velocities(self) -> VectorDouble:
        """
                Get the velocities of all joints.
        
                Returns
                -------
                list of float
                    Velocities in rad/s for arm joints and m/s for the gripper joint.
        """
    def get_arm_accelerations(self) -> VectorDouble:
        """
                Get the accelerations of the arm joints.
        
                Returns
                -------
                list of float
                    Accelerations in rad/s^2.
        """
    def get_arm_compensation_efforts(self) -> VectorDouble:
        """
                Get the compensation efforts of the arm joints.
        
                Returns
                -------
                list of float
                    Compensation efforts in Nm.
        """
    def get_arm_driver_temperatures(self) -> VectorDouble:
        """
                Get the driver temperatures of the arm joints.
        
                Returns
                -------
                list of float
                    Driver temperatures in C.
        """
    def get_arm_efforts(self) -> VectorDouble:
        """
                Get the efforts of the arm joints.
        
                Returns
                -------
                list of float
                    Efforts in Nm.
        """
    def get_arm_external_efforts(self) -> VectorDouble:
        """
                Get the external efforts of the arm joints.
        
                Returns
                -------
                list of float
                    External efforts in Nm.
        """
    def get_arm_positions(self) -> VectorDouble:
        """
                Get the positions of the arm joints.
        
                Returns
                -------
                list of float
                    Positions in rad.
        """
    def get_arm_rotor_temperatures(self) -> VectorDouble:
        """
                Get the rotor temperatures of the arm joints.
        
                Returns
                -------
                list of float
                    Rotor temperatures in C.
        """
    def get_arm_velocities(self) -> VectorDouble:
        """
                Get the velocities of the arm joints.
        
                Returns
                -------
                list of float
                    Velocities in rad.
        """
    def get_cartesian_accelerations(self) -> ArrayDouble6:
        """
                Get the Cartesian accelerations.
        
                Returns
                -------
                list of float
                    Spatial acceleration of the end effector frame with respect to the base frame
                    measured in the base frame in m/s^2 and rad/s^2.
        
                Notes
                -----
                The first 3 elements are the linear acceleration and the last 3 elements are the angular
                acceleration.
        """
    def get_cartesian_external_efforts(self) -> ArrayDouble6:
        """
                Get the Cartesian external efforts.
        
                Returns
                -------
                list of float
                    Spatial external efforts applied to the end effector frame measured in the base frame
                    in N and Nm.
        
                Notes
                -----
                The first 3 elements are the force and the last 3 elements are the torque.
        """
    def get_cartesian_positions(self) -> ArrayDouble6:
        """
                Get the Cartesian positions.
        
                Returns
                -------
                list of float
                    Spatial position of the end effector frame measured in the base frame in m and rad.
        
                Notes
                -----
                The first 3 elements are the translation and the last 3 elements are the angle-axis
                representation of the rotation.
        """
    def get_cartesian_velocities(self) -> ArrayDouble6:
        """
                Get the Cartesian velocities.
        
                Returns
                -------
                list of float
                    Spatial velocity of the end effector frame with respect to the base frame
                    measured in the base frame in m/s and rad/s.
        
                Notes
                -----
                The first 3 elements are the linear velocity and the last 3 elements are the angular
                velocity.
        """
    def get_compensation_efforts(self) -> VectorDouble:
        """
                Get the compensation efforts.
        
                Returns
                -------
                list of float
                    Compensation efforts in Nm for arm joints and N for the gripper joint.
        """
    def get_controller_version(self) -> str:
        """
                Get the controller firmware version.
        
                Returns
                -------
                str
                    Controller firmware version.
        """
    def get_dns(self) -> str:
        """
                Get the DNS.
        
                Returns
                -------
                str
                    DNS address.
        """
    def get_driver_version(self) -> str:
        """
                Get the driver version.
        
                Returns
                -------
                str
                    Driver version.
        """
    def get_effort_corrections(self) -> VectorDouble:
        """
                Get the effort corrections.
        
                Returns
                -------
                list of float
                    Effort corrections in motor effort unit / Nm or N.
        """
    def get_efforts(self) -> VectorDouble:
        """
                Get the efforts.
        
                Returns
                -------
                list of float
                    Efforts in Nm for arm joints and N for the gripper joint.
        """
    def get_end_effector(self) -> EndEffector:
        """
                Get the end effector mass properties.
        
                Returns
                -------
                EndEffector
                    The end effector properties.
        """
    def get_error_information(self) -> str:
        """
                Get the error information of the robot.
        
                Returns
                -------
                str
                    Error information.
        """
    def get_external_efforts(self) -> VectorDouble:
        """
                Get the external efforts.
        
                Returns
                -------
                list of float
                    External efforts in Nm for arm joints and N for the gripper joint.
        """
    def get_factory_reset_flag(self) -> bool:
        """
                Get the factory reset flag.
        
                Returns
                -------
                bool
                    True if the configurations will be reset to factory defaults at the next startup,
                    False otherwise.
        """
    def get_friction_constant_terms(self) -> VectorDouble:
        """
                Get the friction constant terms.
        
                Returns
                -------
                list of float
                    Friction constant terms in Nm for arm joints and N for the gripper joint.
        """
    def get_friction_coulomb_coefs(self) -> VectorDouble:
        """
                Get the friction coulomb coefs.
        
                Returns
                -------
                list of float
                    Friction coulomb coefs in Nm/Nm for arm joints and N/N for the gripper joint.
        """
    def get_friction_transition_velocities(self) -> VectorDouble:
        """
                Get the friction transition velocities.
        
                Returns
                -------
                list of float
                    Friction transition velocities in rad/s for arm joints and m/s for the gripper joint.
        """
    def get_friction_viscous_coefs(self) -> VectorDouble:
        """
                Get the friction viscous coefs.
        
                Returns
                -------
                list of float
                    Friction viscous coefs in Nm/(rad/s) for arm joints and N/(m/s) for the gripper joint.
        """
    def get_gateway(self) -> str:
        """
                Get the gateway.
        
                Returns
                -------
                str
                    Gateway address.
        """
    def get_gripper_acceleration(self) -> float:
        """
                Get the acceleration of the gripper.
        
                Returns
                -------
                float
                    Acceleration in m/s^2.
        """
    def get_gripper_compensation_effort(self) -> float:
        """
                Get the compensation effort of the gripper.
        
                Returns
                -------
                float
                    Compensation effort in N.
        """
    def get_gripper_driver_temperature(self) -> float:
        """
                Get the driver temperature of the gripper.
        
                Returns
                -------
                float
                    Driver temperature in C.
        """
    def get_gripper_effort(self) -> float:
        """
                Get the effort of the gripper.
        
                Returns
                -------
                float
                    Effort in N.
        """
    def get_gripper_external_effort(self) -> float:
        """
                Get the external effort of the gripper.
        
                Returns
                -------
                float
                    External effort in N.
        """
    def get_gripper_position(self) -> float:
        """
                Get the position of the gripper.
        
                Returns
                -------
                float
                    Position in m.
        """
    def get_gripper_rotor_temperature(self) -> float:
        """
                Get the rotor temperature of the gripper.
        
                Returns
                -------
                float
                    Rotor temperature in C.
        """
    def get_gripper_velocity(self) -> float:
        """
                Get the velocity of the gripper.
        
                Returns
                -------
                float
                    Velocity in m/s.
        """
    def get_ip_method(self) -> IPMethod:
        """
                Get the IP method.
        
                Returns
                -------
                IPMethod
                    IP method.
        """
    def get_is_configured(self) -> bool:
        """
                Get the configured status of the robot
        
                Returns
                -------
                bool
                    True if the robot is configured, False otherwise.
        """
    def get_joint_acceleration(self, joint_index: typing.SupportsInt) -> float:
        """
                Get the acceleration of a joint.
        
                Parameters
                ----------
                joint_index : int
                    The index of the joint in [0, num_joints - 1].
        
                Returns
                -------
                float
                    Acceleration in rad/s^2 for arm joints and m/s^2 for the gripper joint.
        """
    def get_joint_characteristics(self) -> list[JointCharacteristic]:
        """
                Get the joint characteristics.
        
                Returns
                -------
                list of JointCharacteristic
                    Joint characteristics.
        """
    def get_joint_compensation_effort(self, joint_index: typing.SupportsInt) -> float:
        """
                Get the compensation effort of a joint.
        
                Parameters
                ----------
                joint_index : int
                    The index of the joint in [0, num_joints - 1].
        
                Returns
                -------
                float
                    Compensation effort in Nm for arm joints and N for the gripper joint.
        """
    def get_joint_driver_temperature(self, joint_index: typing.SupportsInt) -> float:
        """
                Get the driver temperature of a joint.
        
                Parameters
                ----------
                joint_index : int
                    The index of the joint in [0, num_joints - 1].
        
                Returns
                -------
                float
                    Driver temperature in C.
        """
    def get_joint_effort(self, joint_index: typing.SupportsInt) -> float:
        """
                Get the effort of a joint.
        
                Parameters
                ----------
                joint_index : int
                    The index of the joint in [0, num_joints - 1].
        
                Returns
                -------
                float
                    Effort in Nm for arm joints and N for the gripper joint.
        """
    def get_joint_external_effort(self, joint_index: typing.SupportsInt) -> float:
        """
                Get the external effort of a joint.
        
                Parameters
                ----------
                joint_index : int
                    The index of the joint in [0, num_joints - 1].
        
                Returns
                -------
                float
                    External effort in Nm for arm joints and N for the gripper joint.
        """
    def get_joint_limits(self) -> list[JointLimit]:
        """
                Get the joint limits.
        
                Returns
                -------
                list of JointLimit
                    Joint limits of all joints.
        """
    def get_joint_position(self, joint_index: typing.SupportsInt) -> float:
        """
                Get the position of a joint.
        
                Parameters
                ----------
                joint_index : int
                    The index of the joint in [0, num_joints - 1].
        
                Returns
                -------
                float
                    Position in rad for arm joints and m for the gripper joint.
        """
    def get_joint_rotor_temperature(self, joint_index: typing.SupportsInt) -> float:
        """
                Get the rotor temperature of a joint.
        
                Parameters
                ----------
                joint_index : int
                    The index of the joint in [0, num_joints - 1].
        
                Returns
                -------
                float
                    Rotor temperature in C.
        """
    def get_joint_velocity(self, joint_index: typing.SupportsInt) -> float:
        """
                Get the velocity of a joint.
        
                Parameters
                ----------
                joint_index : int
                    The index of the joint in [0, num_joints - 1].
        
                Returns
                -------
                float
                    Velocity in rad/s for arm joints and m/s for the gripper joint.
        """
    def get_manual_ip(self) -> str:
        """
                Get the manual IP.
        
                Returns
                -------
                str
                    Manual IP address.
        """
    def get_modes(self) -> list[Mode]:
        """
                Get the modes.
        
                Returns
                -------
                list of Mode
                    Modes of all joints.
        """
    def get_motor_parameters(self) -> list[dict[Mode, MotorParameter]]:
        """
                Get the motor parameters.
        
                Returns
                -------
                list of dict of MotorParameter
                    Motor parameters of all modes of all joints.
        """
    def get_num_joints(self) -> int:
        """
                Get the number of joints.
        
                Returns
                -------
                int
                    Number of joints.
        """
    def get_position_offsets(self) -> VectorDouble:
        """
                Get the position offsets.
        
                Returns
                -------
                list of float
                    Position offsets in rad for arm joints and m for the gripper joint.
        """
    def get_positions(self) -> VectorDouble:
        """
                Get the positions.
        
                Returns
                -------
                list of float
                    Positions in rad for arm joints and m for the gripper joint.
        """
    def get_robot_output(self) -> RobotOutput:
        """
                Get the robot output.
        
                Returns
                -------
                RobotOutput
                    Robot output.
        """
    def get_subnet(self) -> str:
        """
                Get the subnet.
        
                Returns
                -------
                str
                    Subnet address.
        """
    def get_velocities(self) -> VectorDouble:
        """
                Get the velocities.
        
                Returns
                -------
                list of float
                    Velocities in rad/s for arm joints and m/s for the gripper joint.
        """
    def load_configs_from_file(self, file_path: str) -> None:
        """
                Load configurations from a YAML file and set them.
        
                Parameters
                ----------
                file_path : str
                    The file path to load the configurations.
        """
    def reboot_controller(self) -> None:
        """
                Reboot the controller and cleanup the driver.
        
                Notes
                -----
                This function is a wrapper for cleanup(true).
        """
    def save_configs_to_file(self, file_path: str) -> None:
        """
                Save configurations to a YAML file.
        
                Parameters
                ----------
                file_path : str
                    The file path to store the configurations.
        """
    def set_algorithm_parameter(self, algorithm_parameter: AlgorithmParameter) -> None:
        """
                Set the algorithm parameter.
        
                Parameters
                ----------
                algorithm_parameter : AlgorithmParameter
                    Parameter used for robotic algorithms.
        """
    def set_all_efforts(self, goal_efforts: VectorDouble, goal_time: typing.SupportsFloat = 2.0, blocking: bool = True) -> None:
        """
                Set the efforts of all joints.
        
                Parameters
                ----------
                goal_efforts : list of float
                    Efforts in Nm for arm joints and N for the gripper joint.
                goal_time : float, optional
                    Goal time in s when the goal efforts should be reached, default is 2.0s.
                blocking : bool, optional
                    Whether to block until the goal efforts are reached, default is true.
        
                Notes
                -----
                The size of the vectors should be equal to the number of joints.
        """
    def set_all_external_efforts(self, goal_external_efforts: VectorDouble, goal_time: typing.SupportsFloat = 2.0, blocking: bool = True) -> None:
        """
                Set the external efforts of all joints.
        
                Parameters
                ----------
                goal_external_efforts : list of float
                    External efforts in Nm for arm joints and N for the gripper joint.
                goal_time : float, optional
                    Goal time in s when the goal external efforts should be reached, default is 2.0s.
                blocking : bool, optional
                    Whether to block until the goal external efforts are reached, default is true.
        
                Notes
                -----
                The size of the vectors should be equal to the number of joints.
        """
    def set_all_modes(self, mode: Mode = ...) -> None:
        """
                Set all joints to the same mode.
        
                Parameters
                ----------
                mode : Mode
                    Mode for all joints, one of
                    - Mode.idle
                    - Mode.position
                    - Mode.velocity
                    - Mode.external_effort
                    - Mode.effort
        """
    def set_all_positions(self, goal_positions: VectorDouble, goal_time: typing.SupportsFloat = 2.0, blocking: bool = True, goal_feedforward_velocities: trossen_arm.VectorDouble | None = None, goal_feedforward_accelerations: trossen_arm.VectorDouble | None = None) -> None:
        """
                Set the positions of all joints.
        
                Parameters
                ----------
                goal_positions : list of float
                    Positions in rad for arm joints and m for the gripper joint.
                goal_time : float, optional
                    Goal time in s when the goal positions should be reached, default is 2.0s.
                blocking : bool, optional
                    Whether to block until the goal positions are reached, default is true.
                goal_feedforward_velocities : list of float, optional
                    Feedforward velocities in rad/s for arm joints and m/s for the gripper joint,
                    default is zeros.
                goal_feedforward_accelerations : list of float, optional
                    Feedforward accelerations in rad/s^2 for arm joints and m/s^2 for the gripper joint,
                    default is zeros.
        
                Notes
                -----
                The size of the vectors should be equal to the number of joints.
        """
    def set_all_velocities(self, goal_velocities: VectorDouble, goal_time: typing.SupportsFloat = 2.0, blocking: bool = True, goal_feedforward_accelerations: trossen_arm.VectorDouble | None = None) -> None:
        """
                Set the velocities of all joints.
        
                Parameters
                ----------
                goal_velocities : list of float
                    Velocities in rad/s for arm joints and m/s for the gripper joint.
                goal_time : float, optional
                    Goal time in s when the goal velocities should be reached, default is 2.0s.
                blocking : bool, optional
                    Whether to block until the goal velocities are reached, default is true.
                goal_feedforward_accelerations : list of float, optional
                    Feedforward accelerations in rad/s^2 for arm joints and m/s^2 for the gripper joint,
                    default is zeros.
        
                Notes
                -----
                The size of the vectors should be equal to the number of joints.
        """
    def set_arm_efforts(self, goal_efforts: VectorDouble, goal_time: typing.SupportsFloat = 2.0, blocking: bool = True) -> None:
        """
                Set the efforts of the arm joints.
        
                Parameters
                ----------
                goal_efforts : list of float
                    Efforts in Nm.
                goal_time : float, optional
                    Goal time in s when the goal efforts should be reached, default is 2.0s.
                blocking : bool, optional
                    Whether to block until the goal efforts are reached, default is true.
        
                Notes
                -----
                The size of the vectors should be equal to the number of arm joints.
        """
    def set_arm_external_efforts(self, goal_external_efforts: VectorDouble, goal_time: typing.SupportsFloat = 2.0, blocking: bool = True) -> None:
        """
                Set the external efforts of the arm joints.
        
                Parameters
                ----------
                goal_external_efforts : list of float
                    External efforts in Nm.
                goal_time : float, optional
                    Goal time in s when the goal external efforts should be reached, default is 2.0s.
                blocking : bool, optional
                    Whether to block until the goal external efforts are reached, default is true.
        
                Notes
                -----
                The size of the vectors should be equal to the number of arm joints.
        """
    def set_arm_modes(self, mode: Mode = ...) -> None:
        """
                Set the mode of the arm joints.
        
                Parameters
                ----------
                mode : Mode
                    Mode for the arm joints, one of
                    - Mode.idle
                    - Mode.position
                    - Mode.velocity
                    - Mode.external_effort
                    - Mode.effort
        
                Notes
                -----
                This method does not change the gripper joint's mode.
        """
    def set_arm_positions(self, goal_positions: VectorDouble, goal_time: typing.SupportsFloat = 2.0, blocking: bool = True, goal_feedforward_velocities: trossen_arm.VectorDouble | None = None, goal_feedforward_accelerations: trossen_arm.VectorDouble | None = None) -> None:
        """
                Set the positions of the arm joints.
        
                Parameters
                ----------
                goal_positions : list of float
                    Positions in rad.
                goal_time : float, optional
                    Goal time in s when the goal positions should be reached, default is 2.0s.
                blocking : bool, optional
                    Whether to block until the goal positions are reached, default is true.
                goal_feedforward_velocities : list of float, optional
                    Feedforward velocities in rad/s, default is zeros.
                goal_feedforward_accelerations : list of float, optional
                    Feedforward accelerations in rad/s^2, default is zeros.
        
                Notes
                -----
                The size of the vectors should be equal to the number of arm joints.
        """
    def set_arm_velocities(self, goal_velocities: VectorDouble, goal_time: typing.SupportsFloat = 2.0, blocking: bool = True, goal_feedforward_accelerations: trossen_arm.VectorDouble | None = None) -> None:
        """
                Set the velocities of the arm joints.
        
                Parameters
                ----------
                goal_velocities : list of float
                    Velocities in rad.
                goal_time : float, optional
                    Goal time in s when the goal velocities should be reached, default is 2.0s.
                blocking : bool, optional
                    Whether to block until the goal velocities are reached, default is true.
                goal_feedforward_accelerations : list of float, optional
                    Feedforward accelerations in rad/s^2, default is zeros.
        
                Notes
                -----
                The size of the vectors should be equal to the number of arm joints.
        """
    def set_cartesian_external_efforts(self, goal_external_efforts: ArrayDouble6, interpolation_space: InterpolationSpace, goal_time: typing.SupportsFloat = 2.0, blocking: bool = True) -> None:
        """
                Set the external effort of the end effector in Cartesian space.
        
                Parameters
                ----------
                goal_external_efforts : list of float
                    Spatial external efforts applied to the end effector frame measured in the base frame
                    in N and Nm.
                interpolation_space : InterpolationSpace
                    Interpolation space, one of InterpolationSpace.joint or InterpolationSpace.cartesian.
                goal_time : float, optional
                    Goal time in s when the goal external efforts should be reached, default is 2.0s.
                blocking : bool, optional
                    Whether to block until the goal external efforts are reached, default is true.
        
                Notes
                -----
                The first 3 elements of the goal_external_efforts are the force and the last 3 elements are
                the torque.
        """
    def set_cartesian_positions(self, goal_positions: ArrayDouble6, interpolation_space: InterpolationSpace, goal_time: typing.SupportsFloat = 2.0, blocking: bool = True, goal_feedforward_velocities: trossen_arm.ArrayDouble6 | None = None, goal_feedforward_accelerations: trossen_arm.ArrayDouble6 | None = None, num_trajectory_check_samples: typing.SupportsInt = 1000) -> None:
        """
                Set the position of the end effector in Cartesian space.
        
                Parameters
                ----------
                goal_positions : list of float
                    Spatial position of the end effector frame measured in the base frame in m and rad.
                interpolation_space : InterpolationSpace
                    Interpolation space, one of InterpolationSpace.joint or InterpolationSpace.cartesian.
                goal_time : float, optional
                    Goal time in s when the goal positions should be reached, default is 2.0s.
                blocking : bool, optional
                    Whether to block until the goal positions are reached, default is true.
                goal_feedforward_velocities : list of float, optional
                    Spatial velocity of the end effector frame with respect to the base frame measured in
                    the base frame in m/s and rad/s, default zeros.
                goal_feedforward_accelerations : list of float, optional
                    Spatial acceleration of the end effector frame with respect to the base frame
                    measured in the base frame in m/s^2 and rad/s^2, default zeros.
                num_trajectory_check_samples : int, optional
                    Number of evenly sampled time steps to check trajectory feasibility, default 1000.
        
                Notes
                -----
                The first 3 elements of the goal_positions are the translation and the last 3 elements are
                the angle-axis representation of the rotation.
        
                The first 3 elements of the goal_feedforward_velocities are the linear velocity and the last
                3 elements are the angular velocity.
        
                The first 3 elements of the goal_feedforward_accelerations are the linear acceleration and
                the last 3 elements are the angular acceleration.
        """
    def set_cartesian_velocities(self, goal_velocities: ArrayDouble6, interpolation_space: InterpolationSpace, goal_time: typing.SupportsFloat = 2.0, blocking: bool = True, goal_feedforward_accelerations: trossen_arm.ArrayDouble6 | None = None) -> None:
        """
                Set the velocity of the end effector in Cartesian space.
        
                Parameters
                ----------
                goal_velocities : list of float
                    Spatial velocity of the end effector frame with respect to the base frame measured in
                    the base frame in m/s and rad/s.
                interpolation_space : InterpolationSpace
                    Interpolation space, one of InterpolationSpace.joint or InterpolationSpace.cartesian.
                goal_time : float, optional
                    Goal time in s when the goal velocities should be reached, default is 2.0s.
                blocking : bool, optional
                    Whether to block until the goal velocities are reached, default is true.
                goal_feedforward_accelerations : list of float, optional
                    Spatial acceleration of the end effector frame with respect to the base frame
                    measured in the base frame in m/s^2 and rad/s^2, default zeros.
        
                Notes
                -----
                The first 3 elements of the goal_velocities are the linear velocity and the last 3 elements
                are the angular velocity.
        
                The first 3 elements of the goal_feedforward_accelerations are the linear acceleration and
                the last 3 elements are the angular acceleration.
        """
    def set_dns(self, dns: str = '8.8.8.8') -> None:
        """
                Set the DNS.
        
                Parameters
                ----------
                dns : str
                    DNS address.
        """
    def set_effort_corrections(self, effort_corrections: VectorDouble) -> None:
        """
                Set the effort corrections.
        
                Parameters
                ----------
                effort_corrections : list of float
                    Effort corrections in motor effort unit / Nm or N.
        
                Notes
                -----
                This configuration is used to map the efforts in Nm or N to the motor effort unit,
                i.e., effort_correction = motor effort unit / Nm or N.
        
                The size of the vector should be equal to the number of joints.
        
                Each element in the vector should be within the range [0.2, 5.0].
        """
    def set_end_effector(self, end_effector: EndEffector) -> None:
        """
                Set the end effector properties.
        
                Parameters
                ----------
                end_effector : EndEffector
                    The end effector properties.
        """
    def set_factory_reset_flag(self, flag: bool = True) -> None:
        """
                Set the factory reset flag.
        
                Parameters
                ----------
                flag : bool
                    Whether to reset the configurations to factory defaults at the next startup.
        """
    def set_friction_constant_terms(self, friction_constant_terms: VectorDouble) -> None:
        """
                Set the friction constant terms.
        
                Parameters
                ----------
                friction_constant_terms : list of float
                    Friction constant terms in Nm for arm joints and N for the gripper joint.
        
                Notes
                -----
                The size of the vector should be equal to the number of joints.
        """
    def set_friction_coulomb_coefs(self, friction_coulomb_coefs: VectorDouble) -> None:
        """
                Set the friction coulomb coefs.
        
                Parameters
                ----------
                friction_coulomb_coefs : list of float
                    Friction coulomb coefs in Nm/Nm for arm joints and N/N for the gripper joint.
        
                Notes
                -----
                The size of the vector should be equal to the number of joints.
        """
    def set_friction_transition_velocities(self, friction_transition_velocities: VectorDouble) -> None:
        """
                Set the friction transition velocities.
        
                Parameters
                ----------
                friction_transition_velocities : list of float
                    Friction transition velocities in rad/s for arm joints and m/s for the gripper joint.
        
                Notes
                -----
                The size of the vector should be equal to the number of joints.
        
                Each element in the vector should be positive.
        """
    def set_friction_viscous_coefs(self, friction_viscous_coefs: VectorDouble) -> None:
        """
                Set the friction viscous coefs.
        
                Parameters
                ----------
                friction_viscous_coefs : list of float
                    Friction viscous coefs in Nm/(rad/s) for arm joints and N/(m/s) for the gripper joint.
        
                Notes
                -----
                The size of the vector should be equal to the number of joints.
        """
    def set_gateway(self, gateway: str = '192.168.1.1') -> None:
        """
                Set the gateway.
        
                Parameters
                ----------
                gateway : str
                    Gateway address.
        """
    def set_gripper_effort(self, goal_effort: typing.SupportsFloat, goal_time: typing.SupportsFloat = 2.0, blocking: bool = True) -> None:
        """
                Set the effort of the gripper.
        
                Parameters
                ----------
                goal_effort : float
                    Effort in N.
                goal_time : float, optional
                    Goal time in s when the goal effort should be reached, default is 2.0s.
                blocking : bool, optional
                    Whether to block until the goal effort is reached, default is true.
        """
    def set_gripper_external_effort(self, goal_external_effort: typing.SupportsFloat, goal_time: typing.SupportsFloat = 2.0, blocking: bool = True) -> None:
        """
                Set the external effort of the gripper.
        
                Parameters
                ----------
                goal_external_effort : float
                    External effort in N.
                goal_time : float, optional
                    Goal time in s when the goal external effort should be reached, default is 2.0s.
                blocking : bool, optional
                    Whether to block until the goal external effort is reached, default is true.
        """
    def set_gripper_mode(self, mode: Mode = ...) -> None:
        """
                Set the mode of the gripper joint.
        
                Parameters
                ----------
                mode : Mode
                    Mode for the gripper joint, one of
                    - Mode.idle
                    - Mode.position
                    - Mode.velocity
                    - Mode.external_effort
                    - Mode.effort
        
                Notes
                -----
                This method does not change the arm joints' mode.
        """
    def set_gripper_position(self, goal_position: typing.SupportsFloat, goal_time: typing.SupportsFloat = 2.0, blocking: bool = True, goal_feedforward_velocity: typing.SupportsFloat = 0.0, goal_feedforward_acceleration: typing.SupportsFloat = 0.0) -> None:
        """
                Set the position of the gripper.
        
                Parameters
                ----------
                goal_position : float
                    Position in m.
                goal_time : float, optional
                    Goal time in s when the goal position should be reached, default is 2.0s.
                blocking : bool, optional
                    Whether to block until the goal position is reached, default is true.
                goal_feedforward_velocity : float, optional
                    Feedforward velocity in m/s, default is zero.
                goal_feedforward_acceleration : float, optional
                    Feedforward acceleration in m/s^2, default is zero.
        """
    def set_gripper_velocity(self, goal_velocity: typing.SupportsFloat, goal_time: typing.SupportsFloat = 2.0, blocking: bool = True, goal_feedforward_acceleration: typing.SupportsFloat = 0.0) -> None:
        """
                Set the velocity of the gripper.
        
                Parameters
                ----------
                goal_velocity : float
                    Velocity in m/s.
                goal_time : float, optional
                    Goal time in s when the goal velocity should be reached, default is 2.0s.
                blocking : bool, optional
                    Whether to block until the goal velocity is reached, default is true.
                goal_feedforward_acceleration : float, optional
                    Feedforward acceleration in m/s^2, default is zero.
        """
    def set_ip_method(self, method: IPMethod = ...) -> None:
        """
                Set the IP method.
        
                Parameters
                ----------
                method : IPMethod
                    The IP method to set, one of IPMethod.manual or IPMethod.dhcp.
        """
    def set_joint_characteristics(self, joint_characteristics: collections.abc.Sequence[JointCharacteristic]) -> None:
        """
                Set the joint characteristics.
        
                Parameters
                ----------
                joint_characteristics : list of JointCharacteristic
                    Joint characteristics.
        
                Notes
                -----
                The size of the vector should be equal to the number of joints.
        
                Some joint characteristics are required to be within the following ranges:
                - effort_correction: [0.2, 5.0]
                - friction_transition_velocity: positive
        """
    def set_joint_effort(self, joint_index: typing.SupportsInt, goal_effort: typing.SupportsFloat, goal_time: typing.SupportsFloat = 2.0, blocking: bool = True) -> None:
        """
                Set the effort of a joint.
        
                Parameters
                ----------
                joint_index : int
                    The index of the joint in [0, num_joints - 1].
                goal_effort : float
                    Effort in Nm for arm joints and N for the gripper joint.
                goal_time : float, optional
                    Goal time in s when the goal effort should be reached, default is 2.0s.
                blocking : bool, optional
                    Whether to block until the goal effort is reached, default is true.
        """
    def set_joint_external_effort(self, joint_index: typing.SupportsInt, goal_external_effort: typing.SupportsFloat, goal_time: typing.SupportsFloat = 2.0, blocking: bool = True) -> None:
        """
                Set the external effort of a joint.
        
                Parameters
                ----------
                joint_index : int
                    The index of the joint in [0, num_joints - 1].
                goal_external_effort : float
                    External effort in Nm for arm joints and N for the gripper joint.
                goal_time : float, optional
                    Goal time in s when the goal external effort should be reached, default is 2.0s.
                blocking : bool, optional
                    Whether to block until the goal external effort is reached, default is true.
        """
    def set_joint_limits(self, joint_limits: collections.abc.Sequence[JointLimit]) -> None:
        """
                Set the joint limits.
        
                Parameters
                ----------
                joint_limits : list of JointLimit
                    Joint limits of all joints.
        """
    def set_joint_modes(self, modes: collections.abc.Sequence[Mode]) -> None:
        """
                Set the modes of each joint.
        
                Parameters
                ----------
                modes : list of Mode
                    Desired modes for each joint, one of
                    - Mode.idle
                    - Mode.position
                    - Mode.velocity
                    - Mode.external_effort
                    - Mode.effort
        
                Notes
                -----
                The size of the vector should be equal to the number of joints.
        """
    def set_joint_position(self, joint_index: typing.SupportsInt, goal_position: typing.SupportsFloat, goal_time: typing.SupportsFloat = 2.0, blocking: bool = True, goal_feedforward_velocity: typing.SupportsFloat = 0.0, goal_feedforward_acceleration: typing.SupportsFloat = 0.0) -> None:
        """
                Set the position of a joint.
        
                Parameters
                ----------
                joint_index : int
                    The index of the joint in [0, num_joints - 1].
                goal_position : float
                    Position in rad for arm joints and m for the gripper joint.
                goal_time : float, optional
                    Goal time in s when the goal position should be reached, default is 2.0s.
                blocking : bool, optional
                    Whether to block until the goal position is reached, default is true.
                goal_feedforward_velocity : float, optional
                    Feedforward velocity in rad/s for arm joints and m/s for the gripper joint,
                    default is zero.
                goal_feedforward_acceleration : float, optional
                    Feedforward acceleration in rad/s^2 for arm joints and m/s^2 for the gripper joint,
                    default is zero.
        """
    def set_joint_velocity(self, joint_index: typing.SupportsInt, goal_velocity: typing.SupportsFloat, goal_time: typing.SupportsFloat = 2.0, blocking: bool = True, goal_feedforward_acceleration: typing.SupportsFloat = 0.0) -> None:
        """
                Set the velocity of a joint.
        
                Parameters
                ----------
                joint_index : int
                    The index of the joint in [0, num_joints - 1].
                goal_velocity : float
                    Velocity in rad/s for arm joints and m/s for the gripper joint.
                goal_time : float, optional
                    Goal time in s when the goal velocity should be reached, default is 2.0s.
                blocking : bool, optional
                    Whether to block until the goal velocity is reached, default is true.
                goal_feedforward_acceleration : float, optional
                    Feedforward acceleration in rad/s^2 for arm joints and m/s^2 for the gripper joint,
                    default is zero.
        """
    def set_manual_ip(self, manual_ip: str = '192.168.1.2') -> None:
        """
                Set the manual IP.
        
                Parameters
                ----------
                manual_ip : str
                    Manual IP address.
        """
    def set_motor_parameters(self, motor_parameters: collections.abc.Sequence[collections.abc.Mapping[Mode, MotorParameter]]) -> None:
        """
                Set the motor parameters.
        
                Parameters
                ----------
                motor_parameters : list of dict of MotorParameter
                    Motor parameters of all modes of all joints.
        """
    def set_position_offsets(self, position_offsets: VectorDouble) -> None:
        """
                Set the position offsets.
        
                Parameters
                ----------
                position_offsets : list of float
                    Position offsets in rad for arm joints and m for the gripper joint.
        
                Notes
                -----
                The size of the vector should be equal to the number of joints.
        """
    def set_subnet(self, subnet: str = '255.255.255.0') -> None:
        """
                Set the subnet.
        
                Parameters
                ----------
                subnet : str
                    Subnet address.
        """
class VectorDouble:
    __hash__: typing.ClassVar[None] = None
    def __bool__(self) -> bool:
        """
        Check whether the list is nonempty
        """
    def __contains__(self, x: typing.SupportsFloat) -> bool:
        """
        Return true the container contains ``x``
        """
    @typing.overload
    def __delitem__(self, arg0: typing.SupportsInt) -> None:
        """
        Delete the list elements at index ``i``
        """
    @typing.overload
    def __delitem__(self, arg0: slice) -> None:
        """
        Delete list elements using a slice object
        """
    def __eq__(self, arg0: VectorDouble) -> bool:
        ...
    @typing.overload
    def __getitem__(self, s: slice) -> VectorDouble:
        """
        Retrieve list elements using a slice object
        """
    @typing.overload
    def __getitem__(self, arg0: typing.SupportsInt) -> float:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: VectorDouble) -> None:
        """
        Copy constructor
        """
    @typing.overload
    def __init__(self, arg0: collections.abc.Iterable) -> None:
        ...
    def __iter__(self) -> collections.abc.Iterator[float]:
        ...
    def __len__(self) -> int:
        ...
    def __ne__(self, arg0: VectorDouble) -> bool:
        ...
    def __repr__(self) -> str:
        """
        Return the canonical string representation of this list.
        """
    @typing.overload
    def __setitem__(self, arg0: typing.SupportsInt, arg1: typing.SupportsFloat) -> None:
        ...
    @typing.overload
    def __setitem__(self, arg0: slice, arg1: VectorDouble) -> None:
        """
        Assign list elements using a slice object
        """
    def append(self, x: typing.SupportsFloat) -> None:
        """
        Add an item to the end of the list
        """
    def clear(self) -> None:
        """
        Clear the contents
        """
    def count(self, x: typing.SupportsFloat) -> int:
        """
        Return the number of times ``x`` appears in the list
        """
    @typing.overload
    def extend(self, L: VectorDouble) -> None:
        """
        Extend the list by appending all the items in the given list
        """
    @typing.overload
    def extend(self, L: collections.abc.Iterable) -> None:
        """
        Extend the list by appending all the items in the given list
        """
    def insert(self, i: typing.SupportsInt, x: typing.SupportsFloat) -> None:
        """
        Insert an item at a given position.
        """
    @typing.overload
    def pop(self) -> float:
        """
        Remove and return the last item
        """
    @typing.overload
    def pop(self, i: typing.SupportsInt) -> float:
        """
        Remove and return the item at index ``i``
        """
    def remove(self, x: typing.SupportsFloat) -> None:
        """
        Remove the first item from the list whose value is x. It is an error if there is no such item.
        """
