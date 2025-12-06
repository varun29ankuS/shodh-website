# Implementing Cognitive Memory for Autonomous Robots: Hebbian Learning, Decay, and Consolidation in Production

*A practical guide to building biologically-inspired memory systems for ROS2 robots and edge AI.*

---

Most robots have the memory of a goldfish. They process sensor data, make decisions, and immediately forget everything. The next run starts from scratch. This is fine for simple automation, but it's a problem for autonomous systems that need to learn from experience.

The AI community is focused on making models larger and context windows bigger. But there's a more fundamental question: **how should an autonomous system remember?**

Human memory didn't evolve to store raw data. It evolved to extract patterns, strengthen useful associations, and forget irrelevant details. This architecture is remarkably efficient. And it's surprisingly straightforward to implement.

## The Problem With Current Approaches

Most robotics memory systems fall into two categories:

**1. Raw logging**: Store everything to disk. Query later. This doesn't scale. A LiDAR running at 10Hz generates 600 scans per minute. You end up with terabytes of data and no way to find what matters.

**2. Cloud-based memory**: Send observations to a cloud service, get context back. This fails in the field. A drone operating in a GPS-denied environment can't wait for network round-trips. A warehouse robot can't pause for 200ms to check cloud memory.

Both approaches miss what biological memory actually does: **selective retention based on utility**.

## How Biological Memory Works

The brain uses several mechanisms that we can implement in software:

### Hebbian Learning
"Neurons that fire together, wire together." When two memories are accessed together, the association between them strengthens. In code:

```rust
const LEARNING_RATE: f32 = 0.1;
const LTP_THRESHOLD: u32 = 5;

impl Relationship {
    pub fn strengthen(&mut self) {
        self.activation_count += 1;

        // Hebbian strengthening with diminishing returns
        let delta = LEARNING_RATE * (1.0 - self.strength);
        self.strength = (self.strength + delta).min(1.0);

        // Long-Term Potentiation: after enough activations, becomes permanent
        if !self.potentiated && self.activation_count >= LTP_THRESHOLD {
            self.potentiated = true;
        }
    }
}
```

The key insight: strength increases with co-activation, but with diminishing returns. A connection can't become infinitely strong.

### Activation Decay

Memories that aren't used fade over time. This follows the Ebbinghaus forgetting curve:

```rust
pub fn decay(&mut self) -> bool {
    let days_elapsed = (Utc::now() - self.last_accessed).num_hours() as f64 / 24.0;
    let half_life = BASE_HALF_LIFE_HOURS * (1.0 + self.strength) as f64;

    // Potentiated synapses decay 10x slower
    let effective_half_life = if self.potentiated {
        half_life / 0.1
    } else {
        half_life
    };

    // Exponential decay
    let decay_factor = (-0.693 / effective_half_life * days_elapsed).exp() as f32;
    self.strength *= decay_factor;

    self.strength < MIN_STRENGTH // Prune if too weak
}
```

Strong associations decay slowly. Weak associations fade fast. Potentiated (repeatedly-used) connections resist decay entirely.

### Semantic Consolidation

Raw episodic memories ("detected obstacle at (3.2, 5.1) at 14:32:07") aren't useful long-term. The brain consolidates them into semantic facts ("zone 3 has frequent obstacles"). In code:

```rust
pub struct SemanticConsolidator {
    min_support: usize,    // How many episodes needed to form a fact
    min_age_days: u32,     // How old episodes must be before consolidation
}

pub enum FactType {
    Preference,    // "robot prefers left turns when possible"
    Capability,    // "can traverse 15-degree inclines"
    Relationship,  // "motor temperature correlates with battery drain"
    Procedure,     // "sequence for docking: slow, align, approach"
    Pattern,       // "delivery delays occur on weekends"
}

pub struct SemanticFact {
    pub fact: String,
    pub confidence: f32,
    pub support_count: usize,
    pub fact_type: FactType,
}
```

When multiple episodic memories share a pattern, they consolidate into a fact. Facts that aren't reinforced eventually decay, but well-supported facts persist.

## Three-Tier Memory Hierarchy

Different memory types serve different purposes:

```rust
pub struct MemorySystem {
    working_memory: WorkingMemory,     // Current context, fast access
    session_memory: SessionMemory,     // Current mission/task
    long_term_memory: MemoryStorage,   // Persistent, compressed
}
```

**Working memory** holds the immediate context: current sensor readings, active plan state, recent decisions. Limited capacity, fastest access.

**Session memory** persists for the current mission. If the robot is doing a delivery, session memory holds the destination, route state, and relevant observations.

**Long-term memory** is persistent storage. Old episodic memories are compressed and indexed. Semantic facts live here. Access is slower but capacity is effectively unlimited.

## Salience: What Gets Remembered

Not every observation deserves the same treatment. A normal sensor reading at a known waypoint is less important than an obstacle in an unexpected location. Salience scoring determines priority:

```rust
// Weights from cognitive psychology research
pub const SALIENCE_RECENCY: f32 = 0.3;     // Recent = important
pub const SALIENCE_FREQUENCY: f32 = 0.25;   // Frequently accessed = important
pub const SALIENCE_IMPORTANCE: f32 = 0.25;  // Explicit markers
pub const SALIENCE_SIMILARITY: f32 = 0.2;   // Semantic relevance
```

Higher salience means longer retention, faster retrieval, and stronger associations.

## Implementation for ROS2

Here's how this looks in a ROS2 node:

```python
from shodh_memory import MemorySystem, Position
import rclpy
from rclpy.node import Node

class RobotMemoryNode(Node):
    def __init__(self):
        super().__init__('robot_memory')

        # Initialize memory with persistent storage
        self.memory = MemorySystem("/robot/memory_db")

        self.obstacle_sub = self.create_subscription(
            LaserScan, '/scan', self.obstacle_callback, 10)
        self.state_sub = self.create_subscription(
            RobotState, '/state', self.state_callback, 10)

    def obstacle_callback(self, msg):
        if min(msg.ranges) < 0.5:
            pos = self.get_current_position()

            # Record the observation
            self.memory.record_obstacle(
                distance=min(msg.ranges),
                position=Position(x=pos.x, y=pos.y, z=0.0)
            )

            # Check for patterns
            similar = self.memory.recall(
                f"obstacle near ({pos.x:.0f}, {pos.y:.0f})",
                limit=5
            )
            if len(similar) >= 3:
                self.get_logger().warn(
                    f"Repeated obstacles at ({pos.x}, {pos.y}) - consider rerouting"
                )

    def state_callback(self, msg):
        # Detect failures and record them
        if msg.error_code != 0:
            self.memory.record_failure(
                description=f"Error {msg.error_code}: {msg.error_msg}",
                severity="high" if msg.emergency_stop else "medium",
                root_cause=msg.diagnostic_info
            )
```

The memory system runs in-process. No network calls. Retrieval latency is 15-50ms on typical embedded hardware. The RocksDB storage engine handles persistence efficiently.

## Real-World Benefits

What does this buy you?

**1. Learning from failures**: When the robot encounters an error, it records the context. Next time it approaches similar conditions, the relevant failures surface automatically.

**2. Pattern recognition**: After multiple missions, the consolidator identifies recurring patterns. "Motor 3 overheats when ambient temperature exceeds 35°C" becomes a persistent fact that informs future decisions.

**3. Adaptive routing**: Waypoints with repeated failures automatically get flagged. The planner can incorporate this into path selection.

**4. Reduced debugging time**: When something goes wrong in the field, you have context. Not just logs, but the associations and patterns the robot learned.

## Deployment Footprint

This runs on embedded hardware:

| Component | Size |
|-----------|------|
| Core library | 6MB |
| MiniLM-L6-v2 model | 22MB |
| ONNX Runtime | 50MB |
| **Total** | ~78MB |

Tested on: Raspberry Pi 4, NVIDIA Jetson Nano, x86_64 with 2GB RAM.

## Getting Started

```bash
pip install shodh-memory
```

```python
from shodh_memory import MemorySystem

memory = MemorySystem("./robot_memory")

# Record experiences
memory.record("Navigation succeeded via waypoint 7")
memory.record_failure("Motor stall at high incline", severity="medium")
memory.record_sensor(sensor_name="lidar", readings={"range": 2.3})

# Retrieve relevant memories
results = memory.recall("motor issues on inclines", limit=5)
```

The implementation is open-source: [github.com/varun29ankuS/shodh-memory](https://github.com/varun29ankuS/shodh-memory)

---

The architecture isn't novel. Hebbian learning, activation decay, and memory consolidation have been studied for decades. The contribution is packaging these into a practical system that runs on embedded hardware and integrates with standard robotics stacks.

For autonomous systems that need to learn from experience, this is a starting point. The robots that will succeed in unstructured environments won't just be the ones with the best sensors or the biggest models. They'll be the ones that remember what worked and what didn't.

---

*Tags: robotics, AI, memory, ROS2, edge computing, autonomous systems, Hebbian learning*
