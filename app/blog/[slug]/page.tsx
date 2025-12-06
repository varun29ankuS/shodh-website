'use client';

import { useParams } from 'next/navigation';
import Link from 'next/link';
import { motion } from 'framer-motion';
import { ArrowLeft, Calendar, Clock, Tag } from 'lucide-react';
import { getPostBySlug, getAllPosts } from '@/lib/posts';
import { formatDate, getCategoryColor } from '@/lib/blog';
import CodeBlock from '@/components/CodeBlock';

// Blog post content components
function AgenticAIMemoryPost() {
  return (
    <article className="prose prose-lg dark:prose-invert max-w-none">
      <p className="lead">
        Most AI agents have amnesia. They process a request, respond, and forget everything.
        The next session starts from scratch. This is the "context window problem" that
        limits what autonomous agents can actually accomplish.
      </p>

      <h2>The Problem: Context Windows Are Finite</h2>
      <p>
        LLMs operate within fixed context windows (4K-128K tokens). Long-running agents
        face "context bloat" - the window fills with tool outputs, previous reasoning,
        and conversation history. At some point, you hit the limit and have to drop context.
      </p>
      <p>
        The typical solution is summarization: compress old context into shorter summaries.
        But summarization loses detail. The agent forgets the specific error message that
        caused a failure, the exact configuration that worked, the reasoning behind a decision.
      </p>

      <h2>How Biological Memory Actually Works</h2>
      <p>
        Human memory doesn't work like a context window. It operates through:
      </p>
      <ul>
        <li><strong>Hebbian learning</strong>: "Neurons that fire together, wire together." Associations strengthen with repeated co-activation.</li>
        <li><strong>Activation decay</strong>: Unused memories fade over time following the Ebbinghaus forgetting curve.</li>
        <li><strong>Long-Term Potentiation (LTP)</strong>: After sufficient repetition, synaptic connections become permanent.</li>
        <li><strong>Consolidation</strong>: Episodic memories (events) transform into semantic memories (facts) during sleep.</li>
      </ul>
      <p>
        These mechanisms evolved because they're efficient. You don't need to remember
        every detail of every day - you need to remember what matters.
      </p>

      <h2>Implementing Hebbian Learning in Code</h2>
      <p>
        Here's the core idea: when two memories are retrieved together, strengthen
        the connection between them. When a memory isn't accessed, let it decay.
      </p>

      <CodeBlock
        language="rust"
        filename="graph_memory.rs"
        code={`/// Hebbian synaptic plasticity: "Neurons that fire together, wire together"
/// - Strength increases with co-activation
/// - Strength decays over time without use
/// - After threshold activations, becomes permanent (LTP)

const LEARNING_RATE: f32 = 0.1;
const LTP_THRESHOLD: u32 = 5;
const LTP_DECAY_FACTOR: f32 = 0.1; // Potentiated synapses decay 10x slower

impl Relationship {
    pub fn strengthen(&mut self) {
        self.activation_count += 1;

        // Hebbian strengthening: diminishing returns as strength approaches 1.0
        let delta = LEARNING_RATE * (1.0 - self.strength);
        self.strength = (self.strength + delta).min(1.0);

        // Check for Long-Term Potentiation
        if !self.potentiated && self.activation_count >= LTP_THRESHOLD {
            self.potentiated = true;
        }
    }

    pub fn decay(&mut self) -> bool {
        let days_elapsed = (Utc::now() - self.last_accessed).num_hours() as f64 / 24.0;
        let half_life = BASE_HALF_LIFE_HOURS * (1.0 + self.strength) as f64;

        // Potentiated synapses decay much slower
        let effective_half_life = if self.potentiated {
            half_life / LTP_DECAY_FACTOR as f64
        } else {
            half_life
        };

        let decay_factor = (-0.693 / effective_half_life * days_elapsed).exp() as f32;
        self.strength *= decay_factor;

        self.strength < MIN_STRENGTH // Return true if should be pruned
    }
}`}
      />

      <h2>Three-Tier Memory Hierarchy</h2>
      <p>
        Raw storage isn't enough. You need different tiers for different access patterns:
      </p>
      <ul>
        <li><strong>Working memory</strong>: Fast access, limited capacity (like CPU cache). Current context and active associations.</li>
        <li><strong>Session memory</strong>: Medium-term storage for the current task. Survives context window resets.</li>
        <li><strong>Long-term memory</strong>: Persistent storage in RocksDB. Compressed, indexed by semantic similarity.</li>
      </ul>

      <CodeBlock
        language="rust"
        filename="mod.rs"
        code={`pub struct MemorySystem {
    /// Three-tier memory hierarchy
    working_memory: Arc<RwLock<WorkingMemory>>,
    session_memory: Arc<RwLock<SessionMemory>>,
    long_term_memory: Arc<MemoryStorage>,

    /// Compression pipeline for consolidation
    compressor: CompressionPipeline,

    /// Semantic retrieval with graph associations
    retriever: RetrievalEngine,
}`}
      />

      <h2>Semantic Consolidation: Episodic to Semantic</h2>
      <p>
        Raw episodic memories ("user clicked button X at 3pm") aren't useful long-term.
        Semantic consolidation extracts patterns and converts them into facts:
      </p>

      <CodeBlock
        language="rust"
        filename="compression.rs"
        code={`/// Semantic consolidation: episodic → semantic transformation
/// Like what happens during human sleep
pub struct SemanticConsolidator {
    /// Minimum times a pattern must appear to become a fact
    min_support: usize,
    /// Minimum age before consolidation (days)
    min_age_days: u32,
}

pub enum FactType {
    Preference,    // "prefers dark mode"
    Capability,    // "can handle 10k req/sec"
    Relationship,  // "auth depends on redis"
    Procedure,     // "to deploy, run make release"
    Definition,    // "JWT is JSON Web Token"
    Pattern,       // "errors spike at 3am"
}

pub struct SemanticFact {
    pub fact: String,
    pub confidence: f32,
    pub support_count: usize,  // How many episodic memories support this
    pub source_ids: Vec<String>,
    pub fact_type: FactType,
}`}
      />
      <p>
        Example: if an agent encounters "API timeout when batch &gt; 100" five times,
        the consolidator extracts the fact: "Capability: batch size limited to 100"
        with confidence proportional to the support count.
      </p>

      <h2>Rich Context: Beyond Simple Key-Value</h2>
      <p>
        Context isn't a single thing. The system tracks multiple dimensions:
      </p>

      <CodeBlock
        language="rust"
        filename="context.rs"
        code={`pub struct RichContext {
    pub conversation: ConversationContext,  // Current dialogue state
    pub user: UserContext,                  // User preferences, history
    pub project: ProjectContext,            // Codebase, dependencies
    pub temporal: TemporalContext,          // Time of day, session duration
    pub semantic: SemanticContext,          // Active topics, entities
    pub code: CodeContext,                  // Files, functions in focus
    pub document: DocumentContext,          // Docs being referenced
    pub environment: EnvironmentContext,    // OS, shell, working dir

    pub decay_rate: f32,  // How fast this context loses relevance
    pub embeddings: Option<Vec<f32>>,  // Semantic vector for similarity
}`}
      />
      <p>
        This multi-dimensional context allows retrieval to be nuanced. Looking for
        memories about "authentication"? The system considers not just semantic
        similarity but also which project you're in, what files you're editing,
        and what you've been discussing recently.
      </p>

      <h2>Salience: What Gets Remembered</h2>
      <p>
        Not every piece of information deserves the same treatment. Salience scoring
        determines what gets prioritized during retrieval:
      </p>

      <CodeBlock
        language="rust"
        filename="constants.rs"
        code={`// Salience weights based on cognitive psychology research
pub const SALIENCE_RECENCY_WEIGHT: f32 = 0.3;   // Recent = more salient
pub const SALIENCE_FREQUENCY_WEIGHT: f32 = 0.25; // Frequently accessed = important
pub const SALIENCE_IMPORTANCE_WEIGHT: f32 = 0.25; // Explicit importance markers
pub const SALIENCE_SIMILARITY_WEIGHT: f32 = 0.2;  // Semantic relevance

// Matches Ebbinghaus forgetting curve decay rates
pub const BASE_HALF_LIFE_HOURS: f64 = 24.0;`}
      />

      <h2>Practical Application</h2>
      <p>
        The end result: an agent that remembers what worked, forgets what didn't,
        and builds stronger associations between related concepts over time.
      </p>

      <CodeBlock
        language="python"
        filename="example.py"
        code={`from shodh_memory import MemorySystem

memory = MemorySystem("./agent_memory")

# Record experiences
memory.record("User prefers dark mode", type="Context")
memory.record("API timeout when batch size > 100", type="Error")
memory.record("Switched to streaming for large requests", type="Decision")

# Retrieve relevant memories
results = memory.recall("API performance issues", limit=5)
# Returns: timeout error + streaming decision (high co-activation)`}
      />

      <h2>The Numbers</h2>
      <p>
        Some benchmarks from production use:
      </p>
      <ul>
        <li>50-80ms retrieval latency on 10K+ memories</li>
        <li>~90% token savings vs. stuffing full history into context</li>
        <li>6MB binary, runs on Raspberry Pi 4</li>
      </ul>
      <p>
        The architecture isn't novel - it's based on decades of cognitive science research.
        The contribution is packaging it into a practical system for AI agents.
      </p>
    </article>
  );
}

function EmbodiedAIMemoryPost() {
  return (
    <article className="prose prose-lg dark:prose-invert max-w-none">
      <p className="lead">
        Humanoid robots and autonomous vehicles can't wait 200ms for a cloud round-trip.
        When a drone detects an obstacle at 15m/s, latency isn't a feature request - it's
        the difference between avoiding a collision and becoming one.
      </p>

      <h2>The Cloud Latency Problem</h2>
      <p>
        Most AI memory systems (Mem0, Zep, LangGraph Memory) are designed for chatbots.
        They assume:
      </p>
      <ul>
        <li>Stable internet connectivity</li>
        <li>Latency tolerance in the hundreds of milliseconds</li>
        <li>Centralized storage is acceptable</li>
      </ul>
      <p>
        These assumptions break down for embodied AI. A warehouse robot navigating at 2m/s
        needs sub-100ms decision cycles. A delivery drone operating in rural areas may have
        intermittent connectivity. A surgical robot cannot have its memory dependent on
        network availability.
      </p>

      <h2>Edge-Native Architecture</h2>
      <p>
        Edge-native means the memory system runs entirely on the device:
      </p>

      <CodeBlock
        language="python"
        filename="ros2_node.py"
        code={`from shodh_memory import MemorySystem
import rclpy
from rclpy.node import Node

class RobotMemoryNode(Node):
    def __init__(self):
        super().__init__('robot_memory')

        # Memory runs in-process - no network calls
        self.memory = MemorySystem("/robot/memory_db")

        self.obstacle_sub = self.create_subscription(
            LaserScan, '/scan', self.obstacle_callback, 10)

    def obstacle_callback(self, msg):
        # 5ms to record, 15ms to recall - no cloud dependency
        if min(msg.ranges) < 0.5:
            self.memory.record_obstacle(
                distance=min(msg.ranges),
                position=self.get_position()
            )

            # Check if we've seen similar obstacles before
            similar = self.memory.recall(
                f"obstacle at {self.get_zone()}", limit=3
            )
            if len(similar) > 2:
                self.get_logger().warn("Repeated obstacle zone")`}
      />

      <h2>Why RocksDB, Not SQLite</h2>
      <p>
        RocksDB is an LSM-tree storage engine optimized for write-heavy workloads.
        Robotics memory is write-heavy: constant sensor readings, state updates,
        decision logs. SQLite's B-tree structure would create write amplification.
      </p>
      <p>
        The tradeoff: RocksDB has higher read latency for point queries. But with
        semantic indexing (Vamana HNSW), reads go through the vector index anyway,
        not direct key lookups.
      </p>

      <h2>Memory Consolidation for Robots</h2>
      <p>
        Robots generate massive amounts of data. A LiDAR at 10Hz produces 600 scans
        per minute. You can't store everything. Consolidation compresses episodic
        memories (individual events) into semantic memories (patterns):
      </p>

      <CodeBlock
        language="rust"
        filename="compression.rs"
        code={`/// Consolidation: episodic → semantic transformation
/// Similar to what happens during human sleep
pub struct SemanticConsolidator {
    min_age_days: u32,
    similarity_threshold: f32,
}

impl SemanticConsolidator {
    pub fn consolidate(&self, memories: Vec<Memory>) -> Vec<SemanticFact> {
        // Group similar memories by semantic clustering
        let clusters = self.cluster_by_embedding(memories);

        clusters.into_iter().map(|cluster| {
            SemanticFact {
                // Extract common pattern from cluster
                content: self.extract_pattern(&cluster),
                support_count: cluster.len(),
                confidence: cluster.avg_importance(),
                fact_type: self.classify_fact(&cluster),
            }
        }).collect()
    }
}`}
      />

      <h2>Offline-First, Sync-Optional</h2>
      <p>
        The system is designed to work fully offline. If you want multi-robot
        synchronization, that's an optional layer:
      </p>
      <ul>
        <li>Each robot maintains its own RocksDB instance</li>
        <li>Sync happens opportunistically when connectivity allows</li>
        <li>Conflict resolution uses last-write-wins with vector clocks</li>
      </ul>
      <p>
        This matches how biological systems work: your brain doesn't need WiFi
        to form memories.
      </p>

      <h2>Deployment Footprint</h2>
      <table>
        <thead>
          <tr><th>Component</th><th>Size</th></tr>
        </thead>
        <tbody>
          <tr><td>Core binary</td><td>6MB</td></tr>
          <tr><td>MiniLM-L6-v2 model</td><td>22MB</td></tr>
          <tr><td>ONNX Runtime</td><td>50MB</td></tr>
          <tr><td>Total</td><td>~78MB</td></tr>
        </tbody>
      </table>
      <p>
        Runs on: Raspberry Pi 4, NVIDIA Jetson Nano, any x86_64 with 2GB RAM.
      </p>
    </article>
  );
}

function ROS2TutorialPost() {
  return (
    <article className="prose prose-lg dark:prose-invert max-w-none">
      <p className="lead">
        This is a practical guide to adding persistent memory to ROS2 robots.
        We'll implement a memory node that records failures, tracks waypoints,
        and learns from operational experience.
      </p>

      <h2>Prerequisites</h2>
      <ul>
        <li>ROS2 Humble or later</li>
        <li>Python 3.8+</li>
        <li>shodh-memory: <code>pip install shodh-memory</code></li>
      </ul>

      <h2>Step 1: Create the Memory Node</h2>
      <p>
        We'll create a ROS2 node that wraps the memory system and exposes it
        through services:
      </p>

      <CodeBlock
        language="python"
        filename="memory_node.py"
        code={`#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from shodh_memory import MemorySystem, Position

from robot_interfaces.srv import RecordMemory, RecallMemory
from robot_interfaces.msg import RobotState

class MemoryNode(Node):
    def __init__(self):
        super().__init__('robot_memory')

        # Initialize memory system with persistent storage
        memory_path = self.declare_parameter('memory_path', '/robot/memory').value
        self.memory = MemorySystem(memory_path)
        self.get_logger().info(f'Memory initialized at {memory_path}')

        # Services for recording and recalling
        self.record_srv = self.create_service(
            RecordMemory, 'record_memory', self.record_callback)
        self.recall_srv = self.create_service(
            RecallMemory, 'recall_memory', self.recall_callback)

        # Subscribe to robot state for automatic failure detection
        self.state_sub = self.create_subscription(
            RobotState, '/robot/state', self.state_callback, 10)

        self.last_state = None

    def record_callback(self, request, response):
        memory_id = self.memory.record(
            content=request.content,
            type=request.memory_type or "Observation"
        )
        response.success = True
        response.memory_id = memory_id
        return response

    def recall_callback(self, request, response):
        results = self.memory.recall(request.query, limit=request.limit or 5)
        response.memories = [
            {"id": r.id, "content": r.content, "score": r.score}
            for r in results
        ]
        return response

    def state_callback(self, msg):
        if self.last_state is None:
            self.last_state = msg
            return

        # Detect state transitions that indicate failures
        if self.last_state.operational and not msg.operational:
            self.memory.record_failure(
                description=f"Robot went non-operational: {msg.error_code}",
                severity="high" if msg.emergency_stop else "medium",
                root_cause=msg.error_description
            )
            self.get_logger().warn(f'Recorded failure: {msg.error_code}')

        self.last_state = msg

def main():
    rclpy.init()
    node = MemoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()`}
      />

      <h2>Step 2: Define Service Interfaces</h2>
      <p>
        Create the service definitions in your robot_interfaces package:
      </p>

      <CodeBlock
        language="text"
        filename="srv/RecordMemory.srv"
        code={`string content
string memory_type
---
bool success
string memory_id`}
      />

      <CodeBlock
        language="text"
        filename="srv/RecallMemory.srv"
        code={`string query
int32 limit
---
string[] memories`}
      />

      <h2>Step 3: Waypoint Memory</h2>
      <p>
        Record visited waypoints with success/failure status:
      </p>

      <CodeBlock
        language="python"
        filename="waypoint_tracker.py"
        code={`from shodh_memory import MemorySystem, Position

class WaypointTracker:
    def __init__(self, memory: MemorySystem):
        self.memory = memory

    def record_waypoint(self, name: str, pos: tuple, status: str, notes: str = ""):
        position = Position(x=pos[0], y=pos[1], z=pos[2] if len(pos) > 2 else 0.0)

        self.memory.record_waypoint(
            name=name,
            position=position,
            status=status,  # "reached", "failed", "skipped"
            notes=notes
        )

    def get_problem_waypoints(self) -> list:
        """Find waypoints that frequently fail"""
        results = self.memory.recall("waypoint failed", limit=20)

        # Count failures per waypoint name
        failure_counts = {}
        for r in results:
            # Extract waypoint name from memory content
            if "waypoint" in r.content.lower():
                name = self.extract_waypoint_name(r.content)
                failure_counts[name] = failure_counts.get(name, 0) + 1

        # Return waypoints with 2+ failures
        return [name for name, count in failure_counts.items() if count >= 2]

    def extract_waypoint_name(self, content: str) -> str:
        # Simple extraction - customize for your naming scheme
        import re
        match = re.search(r'waypoint[\\s:]+([\\w-]+)', content, re.I)
        return match.group(1) if match else "unknown"`}
      />

      <h2>Step 4: Learning from Experience</h2>
      <p>
        The Hebbian learning mechanism means frequently co-accessed memories
        strengthen their connections. When you recall "navigation failure" and
        a specific sensor reading appears in multiple results, that association
        gets stronger.
      </p>

      <CodeBlock
        language="python"
        filename="diagnostic_helper.py"
        code={`def diagnose_failure(memory: MemorySystem, symptoms: str) -> list:
    """
    Given current symptoms, find relevant past failures and their resolutions.
    Hebbian learning ensures frequently successful patterns rank higher.
    """
    # Search for similar past issues
    similar_failures = memory.recall(
        f"failure {symptoms}",
        limit=10
    )

    # For each failure, find associated resolutions
    resolutions = []
    for failure in similar_failures:
        # Search for decisions made after this failure
        followup = memory.recall(
            f"decision after {failure.id[:8]}",
            limit=3
        )
        if followup:
            resolutions.append({
                "failure": failure.content,
                "resolution": followup[0].content,
                "confidence": failure.score * followup[0].score
            })

    return sorted(resolutions, key=lambda x: x["confidence"], reverse=True)`}
      />

      <h2>Step 5: Launch Configuration</h2>
      <p>
        Add the memory node to your robot's launch file:
      </p>

      <CodeBlock
        language="python"
        filename="robot_launch.py"
        code={`from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_memory',
            executable='memory_node',
            name='robot_memory',
            parameters=[{
                'memory_path': '/robot/persistent_memory'
            }],
            output='screen'
        ),
        # ... other nodes
    ])`}
      />

      <h2>Memory Maintenance</h2>
      <p>
        The system runs background consolidation automatically. For long-running
        robots, old memories compress into semantic facts:
      </p>
      <ul>
        <li>Individual "navigation failed at waypoint X" events consolidate into "waypoint X has reliability issues"</li>
        <li>Repeated sensor readings consolidate into "LiDAR unreliable in dusty conditions"</li>
        <li>Activation decay prunes memories that haven't been accessed in weeks</li>
      </ul>
      <p>
        The result: a robot that gets more reliable over time because it remembers
        what went wrong and what fixed it.
      </p>
    </article>
  );
}

// Main page component
export default function BlogPostPage() {
  const params = useParams();
  const slug = params.slug as string;
  const post = getPostBySlug(slug);

  if (!post) {
    return (
      <div className="min-h-screen flex items-center justify-center">
        <div className="text-center">
          <h1 className="text-4xl font-bold text-slate-900 dark:text-white mb-4">Post Not Found</h1>
          <Link href="/blog" className="text-orange-600 hover:underline">
            Back to Blog
          </Link>
        </div>
      </div>
    );
  }

  // Render the appropriate content based on slug
  const renderContent = () => {
    switch (slug) {
      case 'agentic-ai-long-term-memory-2025':
        return <AgenticAIMemoryPost />;
      case 'embodied-ai-edge-memory-robotics':
        return <EmbodiedAIMemoryPost />;
      case 'autonomous-robot-memory-ros2-guide':
        return <ROS2TutorialPost />;
      default:
        return <p>Content coming soon...</p>;
    }
  };

  return (
    <div className="min-h-screen bg-gradient-to-b from-slate-50 to-white dark:from-slate-900 dark:to-slate-800">
      <div className="max-w-4xl mx-auto px-4 py-12">
        {/* Back link */}
        <Link
          href="/blog"
          className="inline-flex items-center gap-2 text-slate-600 dark:text-slate-400 hover:text-orange-600 dark:hover:text-orange-400 mb-8"
        >
          <ArrowLeft className="w-4 h-4" />
          Back to Blog
        </Link>

        {/* Header */}
        <motion.header
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.5 }}
          className="mb-12"
        >
          <span className={`inline-block px-3 py-1 rounded-full text-sm font-medium mb-4 ${getCategoryColor(post.category)}`}>
            {post.category}
          </span>

          <h1 className="text-3xl md:text-4xl font-bold text-slate-900 dark:text-white mb-6">
            {post.title}
          </h1>

          <div className="flex flex-wrap items-center gap-4 text-sm text-slate-600 dark:text-slate-400 mb-6">
            <span className="flex items-center gap-1">
              <Calendar className="w-4 h-4" />
              {formatDate(post.date)}
            </span>
            <span className="flex items-center gap-1">
              <Clock className="w-4 h-4" />
              {post.readingTime}
            </span>
            <span>
              By {post.author.name}
              {post.author.role && ` · ${post.author.role}`}
            </span>
          </div>

          <div className="flex flex-wrap gap-2">
            {post.tags.map((tag) => (
              <span
                key={tag}
                className="inline-flex items-center gap-1 px-3 py-1 bg-slate-100 dark:bg-slate-800 text-slate-600 dark:text-slate-300 text-sm rounded-full"
              >
                <Tag className="w-3 h-3" />
                {tag}
              </span>
            ))}
          </div>
        </motion.header>

        {/* Content */}
        <motion.div
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          transition={{ duration: 0.5, delay: 0.2 }}
        >
          {renderContent()}
        </motion.div>

        {/* Footer */}
        <footer className="mt-16 pt-8 border-t border-slate-200 dark:border-slate-700">
          <div className="flex items-center justify-between">
            <Link
              href="/blog"
              className="text-orange-600 dark:text-orange-400 hover:underline"
            >
              ← More posts
            </Link>
            <a
              href="https://github.com/varun29ankuS/shodh-memory"
              target="_blank"
              rel="noopener noreferrer"
              className="text-slate-600 dark:text-slate-400 hover:text-orange-600 dark:hover:text-orange-400"
            >
              View on GitHub →
            </a>
          </div>
        </footer>
      </div>
    </div>
  );
}

// Generate static params for all posts
export function generateStaticParams() {
  const posts = getAllPosts();
  return posts.map((post) => ({
    slug: post.slug,
  }));
}
