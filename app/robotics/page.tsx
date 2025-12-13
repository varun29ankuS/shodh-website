'use client'

import { motion } from 'framer-motion'
import {
  ArrowRight, Bot, Cpu, Database, GitBranch, Network,
  Radio, Zap, Shield, Brain, Workflow, Factory, Sprout, Code,
  Cog, Radar, Plane, Truck
} from 'lucide-react'
import Link from 'next/link'

export default function Robotics() {
  const edgeCapabilities = [
    {
      icon: Brain,
      title: 'Experience-Based Learning',
      description: 'Robots remember what worked and what failed',
      detail: 'Hebbian learning strengthens successful action patterns',
    },
    {
      icon: Database,
      title: '100% Offline Operation',
      description: 'No cloud dependency, no network latency',
      detail: 'Single ~15MB binary runs on ARM/x86 controllers',
    },
    {
      icon: Zap,
      title: 'Real-Time Performance',
      description: 'Sub-millisecond graph operations',
      detail: '<1μs entity lookup, 55ms store with embedding',
    },
    {
      icon: Shield,
      title: 'Air-Gapped Deployments',
      description: 'Defence, industrial, and secure environments',
      detail: 'No external connections required',
    },
  ]

  const useCases = [
    {
      icon: Factory,
      title: 'Industrial Robots',
      description: 'Assembly robots that remember successful procedures, failed attempts, and quality issues',
      example: 'Robot recalls that torque setting X caused failures on part type Y',
    },
    {
      icon: Truck,
      title: 'Warehouse AGVs',
      description: 'Autonomous guided vehicles that learn optimal routes and remember obstacle locations',
      example: 'AGV remembers that aisle 3 has frequent pedestrian traffic at 2pm',
    },
    {
      icon: Plane,
      title: 'Autonomous Drones',
      description: 'Drones that remember flight patterns, weather conditions, and landing challenges',
      example: 'Drone recalls that GPS signal is weak near building X',
    },
    {
      icon: Sprout,
      title: 'Agricultural Robots',
      description: 'Field robots that remember soil conditions, pest locations, and harvest timing',
      example: 'Robot recalls that section B needs different treatment than section A',
    },
  ]

  const benchmarks = [
    { operation: 'Entity Lookup', latency: '<1μs', environment: 'ARM64/x86' },
    { operation: 'Remember (full)', latency: '55-60ms', environment: 'With embedding' },
    { operation: 'Recall (semantic)', latency: '34-58ms', environment: 'Vector search' },
    { operation: 'Recall (tags)', latency: '~1ms', environment: 'Index lookup' },
    { operation: '3-hop Graph Walk', latency: '32μs', environment: 'Association traversal' },
    { operation: 'Hebbian Strengthen', latency: '6μs', environment: 'Synapse update' },
  ]

  const platformSupport = [
    { platform: 'Linux x86_64', status: 'Production', useCase: 'Industrial PCs, servers' },
    { platform: 'Linux ARM64', status: 'Coming Q1 2025', useCase: 'Jetson, Raspberry Pi, drones' },
    { platform: 'Windows x86_64', status: 'Production', useCase: 'Development, industrial HMIs' },
    { platform: 'macOS ARM64', status: 'Production', useCase: 'Development (Apple Silicon)' },
  ]

  return (
    <main className="min-h-screen overflow-hidden">
      {/* Hero */}
      <section className="relative min-h-screen flex items-center justify-center bg-gradient-to-b from-slate-50 via-white to-slate-50 dark:from-slate-950 dark:via-slate-900 dark:to-slate-950 overflow-hidden">
        <div className="absolute inset-0 overflow-hidden">
          <div className="absolute -top-40 -left-40 w-96 h-96 bg-primary/30 rounded-full blur-3xl animate-pulse"></div>
          <div className="absolute top-1/3 -right-40 w-[600px] h-[600px] bg-secondary/25 rounded-full blur-3xl animate-pulse" style={{ animationDelay: '1s' }}></div>
          <div className="absolute -bottom-40 left-1/3 w-[500px] h-[500px] bg-destructive/30 rounded-full blur-3xl animate-pulse" style={{ animationDelay: '2s' }}></div>
        </div>
        <div className="absolute inset-0 bg-grid-slate opacity-40" />
        <div className="absolute inset-0 gradient-mesh"></div>

        <div className="container mx-auto px-4 py-20 relative z-10">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.5 }}
            className="max-w-5xl mx-auto text-center"
          >
            <motion.div
              initial={{ opacity: 0, scale: 0.95 }}
              animate={{ opacity: 1, scale: 1 }}
              transition={{ duration: 0.5, delay: 0.1 }}
              className="inline-flex items-center gap-2 px-4 py-2 bg-primary/10 dark:bg-primary/20 rounded-full mb-8"
            >
              <Bot className="w-4 h-4 text-primary" />
              <span className="text-sm font-semibold text-primary">Edge AI Memory • Robots • Drones • IoT</span>
            </motion.div>

            <h1 className="text-5xl md:text-7xl font-bold mb-6 text-slate-900 dark:text-white">
              Robots That Learn
              <span className="block text-transparent bg-clip-text bg-gradient-to-r from-primary via-secondary to-destructive mt-2">
                From Experience
              </span>
            </h1>

            <p className="text-xl md:text-2xl text-slate-600 dark:text-slate-400 mb-8 max-w-3xl mx-auto">
              Give autonomous systems persistent memory that strengthens with use.
              Hebbian learning on-device. No cloud. No network latency.
              Single binary runs offline on any controller.
            </p>

            <p className="text-base text-slate-500 dark:text-slate-500 mb-12 max-w-2xl mx-auto">
              <strong className="text-primary">~15MB binary</strong> •
              <strong className="text-primary ml-2">&lt;1μs graph ops</strong> •
              <strong className="text-primary ml-2">100% offline</strong> •
              <strong className="text-primary ml-2">ARM64 support coming</strong>
            </p>

            <div className="flex flex-col sm:flex-row gap-4 justify-center mb-12">
              <Link
                href="/memory"
                className="group px-8 py-4 bg-primary hover:bg-primary/90 text-white rounded-lg font-semibold transition-all transform hover:scale-105 flex items-center justify-center gap-2"
              >
                Explore Memory System
                <ArrowRight className="w-5 h-5 group-hover:translate-x-1 transition-transform" />
              </Link>
              <Link
                href="/docs"
                className="px-8 py-4 border-2 border-slate-300 dark:border-slate-700 rounded-lg font-semibold hover:border-primary dark:hover:border-primary transition-colors text-slate-900 dark:text-white"
              >
                Integration Docs
              </Link>
            </div>

            {/* Quick Stats */}
            <div className="grid grid-cols-2 md:grid-cols-4 gap-4 max-w-3xl mx-auto">
              <motion.div
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ delay: 0.3 }}
                className="p-4 bg-white/50 dark:bg-slate-900/50 backdrop-blur-sm rounded-lg border border-slate-200 dark:border-slate-800"
              >
                <div className="text-2xl font-bold text-primary">~15MB</div>
                <div className="text-sm text-slate-600 dark:text-slate-400">Binary Size</div>
              </motion.div>
              <motion.div
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ delay: 0.4 }}
                className="p-4 bg-white/50 dark:bg-slate-900/50 backdrop-blur-sm rounded-lg border border-slate-200 dark:border-slate-800"
              >
                <div className="text-2xl font-bold text-primary">&lt;1μs</div>
                <div className="text-sm text-slate-600 dark:text-slate-400">Graph Lookup</div>
              </motion.div>
              <motion.div
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ delay: 0.5 }}
                className="p-4 bg-white/50 dark:bg-slate-900/50 backdrop-blur-sm rounded-lg border border-slate-200 dark:border-slate-800"
              >
                <div className="text-2xl font-bold text-primary">100%</div>
                <div className="text-sm text-slate-600 dark:text-slate-400">Offline</div>
              </motion.div>
              <motion.div
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ delay: 0.6 }}
                className="p-4 bg-white/50 dark:bg-slate-900/50 backdrop-blur-sm rounded-lg border border-slate-200 dark:border-slate-800"
              >
                <div className="text-2xl font-bold text-primary">RocksDB</div>
                <div className="text-sm text-slate-600 dark:text-slate-400">Durable Storage</div>
              </motion.div>
            </div>
          </motion.div>
        </div>
      </section>

      {/* Why Robots Need Memory */}
      <section className="relative py-24 bg-gradient-to-b from-white to-slate-50 dark:from-slate-950 dark:to-slate-900">
        <div className="absolute inset-0 bg-grid-slate opacity-30"></div>
        <div className="container mx-auto px-4 relative z-10">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              Why Robots Need Memory
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400 max-w-3xl mx-auto">
              Autonomous systems that remember outperform those that don't.
              Experience-based learning without cloud round-trips.
            </p>
          </div>

          <div className="grid md:grid-cols-2 gap-8 max-w-5xl mx-auto mb-16">
            {/* Without Memory */}
            <motion.div
              initial={{ opacity: 0, x: -20 }}
              whileInView={{ opacity: 1, x: 0 }}
              viewport={{ once: true }}
              className="p-6 rounded-xl border-2 border-destructive/30 bg-gradient-to-br from-destructive/5 to-white dark:from-destructive/10 dark:to-slate-900"
            >
              <div className="flex items-center gap-3 mb-4">
                <Shield className="w-7 h-7 text-destructive" />
                <h4 className="text-xl font-bold text-slate-900 dark:text-white">Without Memory</h4>
              </div>
              <div className="space-y-3 text-sm">
                <div className="flex items-start gap-2">
                  <span className="text-destructive font-bold text-lg">x</span>
                  <div>
                    <div className="font-semibold text-slate-900 dark:text-white">Repeats mistakes</div>
                    <div className="text-slate-600 dark:text-slate-400">Same failures over and over</div>
                  </div>
                </div>
                <div className="flex items-start gap-2">
                  <span className="text-destructive font-bold text-lg">x</span>
                  <div>
                    <div className="font-semibold text-slate-900 dark:text-white">No adaptation</div>
                    <div className="text-slate-600 dark:text-slate-400">Can't learn from environment</div>
                  </div>
                </div>
                <div className="flex items-start gap-2">
                  <span className="text-destructive font-bold text-lg">x</span>
                  <div>
                    <div className="font-semibold text-slate-900 dark:text-white">Cloud dependency</div>
                    <div className="text-slate-600 dark:text-slate-400">Network down = robot frozen</div>
                  </div>
                </div>
              </div>
            </motion.div>

            {/* With Shodh Memory */}
            <motion.div
              initial={{ opacity: 0, x: 20 }}
              whileInView={{ opacity: 1, x: 0 }}
              viewport={{ once: true }}
              className="p-6 rounded-xl border-2 border-primary/30 bg-gradient-to-br from-primary/5 to-white dark:from-primary/10 dark:to-slate-900"
            >
              <div className="flex items-center gap-3 mb-4">
                <Brain className="w-7 h-7 text-primary" />
                <h4 className="text-xl font-bold text-slate-900 dark:text-white">With Shodh Memory</h4>
              </div>
              <div className="space-y-3 text-sm">
                <div className="flex items-start gap-2">
                  <span className="text-primary font-bold text-lg">+</span>
                  <div>
                    <div className="font-semibold text-slate-900 dark:text-white">Learns from errors</div>
                    <div className="text-slate-600 dark:text-slate-400">Remembers what failed and why</div>
                  </div>
                </div>
                <div className="flex items-start gap-2">
                  <span className="text-primary font-bold text-lg">+</span>
                  <div>
                    <div className="font-semibold text-slate-900 dark:text-white">Pattern recognition</div>
                    <div className="text-slate-600 dark:text-slate-400">Hebbian learning strengthens successful patterns</div>
                  </div>
                </div>
                <div className="flex items-start gap-2">
                  <span className="text-primary font-bold text-lg">+</span>
                  <div>
                    <div className="font-semibold text-slate-900 dark:text-white">100% offline</div>
                    <div className="text-slate-600 dark:text-slate-400">Runs on robot controller, no network</div>
                  </div>
                </div>
              </div>
            </motion.div>
          </div>

          {/* Key Capabilities */}
          <div className="grid md:grid-cols-2 lg:grid-cols-4 gap-6 max-w-6xl mx-auto">
            {edgeCapabilities.map((capability, index) => (
              <motion.div
                key={capability.title}
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                transition={{ duration: 0.5, delay: index * 0.1 }}
                viewport={{ once: true }}
                className="p-6 rounded-xl border border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900 hover:border-primary dark:hover:border-primary transition-all"
              >
                <div className="w-10 h-10 bg-primary/10 dark:bg-primary/20 rounded-lg flex items-center justify-center mb-4">
                  <capability.icon className="w-5 h-5 text-primary" />
                </div>
                <h3 className="text-lg font-semibold mb-2 text-slate-900 dark:text-white">{capability.title}</h3>
                <p className="text-sm text-slate-600 dark:text-slate-400 mb-2">{capability.description}</p>
                <p className="text-xs text-primary font-medium">{capability.detail}</p>
              </motion.div>
            ))}
          </div>
        </div>
      </section>

      {/* Python SDK for Robotics */}
      <section className="relative py-24 bg-white dark:bg-slate-950">
        <div className="container mx-auto px-4">
          <div className="text-center mb-12">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              Python SDK for Robotics
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400">
              Native PyO3 bindings. Same API as cloud, runs on your controller.
            </p>
          </div>

          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="max-w-4xl mx-auto p-8 bg-slate-900 dark:bg-slate-950 rounded-2xl border border-slate-700"
          >
            <div className="flex items-center gap-2 mb-6 text-slate-400">
              <Code className="w-5 h-5" />
              <span className="text-sm font-semibold">pip install shodh-memory</span>
            </div>
            <pre className="text-slate-100 font-mono text-sm overflow-x-auto">
{`from shodh_memory import Memory

# Initialize on robot controller (fully offline)
memory = Memory(storage_path="./robot_memory")

# Record observations and outcomes
memory.remember(
    "Approach angle 45deg caused gripper slip on part X",
    memory_type="Error",
    tags=["gripper", "part_x", "angle"]
)

memory.remember(
    "Approach angle 30deg successful for part X",
    memory_type="Learning",
    tags=["gripper", "part_x", "angle"]
)

# Before next action, recall relevant experience
results = memory.recall("gripper approach angle part X", limit=5)
for mem in results:
    print(f"{mem.content} (type: {mem.memory_type})")

# Hybrid retrieval - semantic + graph associations
results = memory.recall("gripper failures", mode="hybrid", limit=10)

# ROS2 integration example
import rclpy
from rclpy.node import Node

class RobotMemoryNode(Node):
    def __init__(self):
        super().__init__('memory_node')
        self.memory = Memory(storage_path="/opt/robot/memory")

    def action_callback(self, msg):
        # Record action outcomes
        if msg.success:
            self.memory.remember(
                f"Action {msg.action} succeeded with params {msg.params}",
                memory_type="Learning"
            )
        else:
            self.memory.remember(
                f"Action {msg.action} failed: {msg.error}",
                memory_type="Error"
            )

    def plan_action(self, task):
        # Recall similar past experiences before planning
        experiences = self.memory.recall(task, limit=5)
        return experiences`}
            </pre>

            <div className="grid md:grid-cols-2 gap-4 mt-6">
              <div className="p-4 bg-primary/10 dark:bg-primary/20 rounded-lg">
                <p className="text-xs text-slate-300">
                  <strong className="text-primary">Memory Types:</strong> Error (+0.25), Learning (+0.25),
                  Decision (+0.30), Pattern (+0.20) - importance affects recall priority
                </p>
              </div>
              <div className="p-4 bg-secondary/10 dark:bg-secondary/20 rounded-lg">
                <p className="text-xs text-slate-300">
                  <strong className="text-secondary">Hebbian Learning:</strong> When memories are recalled together,
                  their connection strengthens. 5+ co-activations = permanent edge.
                </p>
              </div>
            </div>
          </motion.div>
        </div>
      </section>

      {/* Use Cases */}
      <section className="relative py-24 bg-gradient-to-b from-slate-50 to-white dark:from-slate-900 dark:to-slate-950">
        <div className="absolute inset-0 overflow-hidden">
          <div className="absolute top-1/4 right-0 w-96 h-96 bg-primary/10 rounded-full blur-3xl"></div>
          <div className="absolute bottom-1/4 left-0 w-96 h-96 bg-secondary/10 rounded-full blur-3xl"></div>
        </div>
        <div className="container mx-auto px-4 relative z-10">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              Built For Real Deployments
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400">
              Autonomous systems that learn and adapt in the field
            </p>
          </div>

          <div className="grid md:grid-cols-2 gap-8 max-w-6xl mx-auto">
            {useCases.map((useCase, index) => (
              <motion.div
                key={useCase.title}
                initial={{ opacity: 0, x: index % 2 === 0 ? -20 : 20 }}
                whileInView={{ opacity: 1, x: 0 }}
                transition={{ duration: 0.5 }}
                viewport={{ once: true }}
                className="p-8 rounded-2xl border border-slate-200 dark:border-slate-800 hover:shadow-xl hover:shadow-primary/10 hover:border-primary dark:hover:border-primary transition-all duration-300 bg-white dark:bg-slate-900"
              >
                <div className="flex items-start gap-4 mb-4">
                  <div className="w-12 h-12 bg-primary/10 dark:bg-primary/20 rounded-lg flex items-center justify-center flex-shrink-0">
                    <useCase.icon className="w-6 h-6 text-primary" />
                  </div>
                  <div>
                    <h3 className="text-xl font-semibold mb-2 text-slate-900 dark:text-white">{useCase.title}</h3>
                    <p className="text-slate-600 dark:text-slate-400">{useCase.description}</p>
                  </div>
                </div>
                <div className="pl-16">
                  <div className="p-4 bg-slate-50 dark:bg-slate-800 rounded-lg border-l-4 border-primary">
                    <p className="text-sm text-slate-700 dark:text-slate-300">
                      <strong>Example:</strong> {useCase.example}
                    </p>
                  </div>
                </div>
              </motion.div>
            ))}
          </div>
        </div>
      </section>

      {/* Benchmarks */}
      <section className="relative py-24 bg-white dark:bg-slate-950">
        <div className="container mx-auto px-4">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              Edge Performance
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400">
              Benchmarked on Intel i7-1355U, release build
            </p>
          </div>

          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="max-w-4xl mx-auto overflow-x-auto"
          >
            <table className="w-full border-collapse">
              <thead>
                <tr className="border-b-2 border-slate-200 dark:border-slate-800">
                  <th className="text-left p-4 text-slate-900 dark:text-white font-semibold">Operation</th>
                  <th className="text-center p-4 text-primary font-semibold">Latency</th>
                  <th className="text-left p-4 text-slate-600 dark:text-slate-400 font-semibold">Notes</th>
                </tr>
              </thead>
              <tbody>
                {benchmarks.map((row, index) => (
                  <tr
                    key={row.operation}
                    className={`border-b border-slate-100 dark:border-slate-800 ${
                      index % 2 === 0 ? 'bg-slate-50 dark:bg-slate-900/50' : ''
                    }`}
                  >
                    <td className="p-4 text-slate-900 dark:text-white font-medium">{row.operation}</td>
                    <td className="p-4 text-center text-primary font-semibold font-mono">{row.latency}</td>
                    <td className="p-4 text-slate-600 dark:text-slate-400 text-sm">{row.environment}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </motion.div>
        </div>
      </section>

      {/* Platform Support */}
      <section className="relative py-24 bg-gradient-to-b from-slate-50 to-white dark:from-slate-900 dark:to-slate-950">
        <div className="container mx-auto px-4">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              Platform Support
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400">
              Single binary for your target architecture
            </p>
          </div>

          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="max-w-4xl mx-auto overflow-x-auto"
          >
            <table className="w-full border-collapse">
              <thead>
                <tr className="border-b-2 border-slate-200 dark:border-slate-800">
                  <th className="text-left p-4 text-slate-900 dark:text-white font-semibold">Platform</th>
                  <th className="text-center p-4 text-slate-900 dark:text-white font-semibold">Status</th>
                  <th className="text-left p-4 text-slate-600 dark:text-slate-400 font-semibold">Use Case</th>
                </tr>
              </thead>
              <tbody>
                {platformSupport.map((row, index) => (
                  <tr
                    key={row.platform}
                    className={`border-b border-slate-100 dark:border-slate-800 ${
                      index % 2 === 0 ? 'bg-slate-50 dark:bg-slate-900/50' : ''
                    }`}
                  >
                    <td className="p-4 text-slate-900 dark:text-white font-medium">{row.platform}</td>
                    <td className="p-4 text-center">
                      <span className={`px-3 py-1 rounded-full text-xs font-semibold ${
                        row.status === 'Production'
                          ? 'bg-primary/10 text-primary'
                          : 'bg-secondary/10 text-secondary'
                      }`}>
                        {row.status}
                      </span>
                    </td>
                    <td className="p-4 text-slate-600 dark:text-slate-400 text-sm">{row.useCase}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </motion.div>

          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="max-w-3xl mx-auto mt-12 p-6 bg-primary/5 dark:bg-primary/10 rounded-xl border border-primary/20"
          >
            <div className="flex items-start gap-4">
              <Radar className="w-8 h-8 text-primary flex-shrink-0 mt-1" />
              <div>
                <h4 className="font-semibold text-slate-900 dark:text-white mb-2">
                  ARM64 Support Coming Soon
                </h4>
                <p className="text-sm text-slate-700 dark:text-slate-300">
                  We're actively working on ARM64 builds for Jetson, Raspberry Pi, and drone flight controllers.
                  x86_64 builds work today on industrial PCs and server-class robot controllers.
                </p>
              </div>
            </div>
          </motion.div>
        </div>
      </section>

      {/* CTA */}
      <section className="relative py-24 bg-gradient-to-br from-primary/10 via-secondary/10 to-destructive/10 dark:from-primary/20 dark:via-secondary/20 dark:to-destructive/20 overflow-hidden">
        <div className="absolute inset-0">
          <div className="absolute top-0 right-0 w-96 h-96 bg-primary/30 rounded-full blur-3xl animate-pulse"></div>
          <div className="absolute bottom-0 left-0 w-96 h-96 bg-secondary/30 rounded-full blur-3xl animate-pulse" style={{ animationDelay: '1s' }}></div>
        </div>
        <div className="container mx-auto px-4 relative z-10">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.5 }}
            viewport={{ once: true }}
            className="max-w-4xl mx-auto text-center"
          >
            <h2 className="text-4xl md:text-5xl font-bold mb-6 text-slate-900 dark:text-white">
              Give Your Robots Memory
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400 mb-8">
              Experience-based learning. 100% offline. Single binary deployment.
            </p>
            <div className="flex flex-col sm:flex-row gap-4 justify-center">
              <Link
                href="/memory"
                className="group px-8 py-4 bg-primary hover:bg-primary/90 text-white rounded-lg font-semibold transition-all transform hover:scale-105 flex items-center justify-center gap-2"
              >
                Explore Memory System
                <ArrowRight className="w-5 h-5 group-hover:translate-x-1 transition-transform" />
              </Link>
              <a
                href="https://github.com/varun29ankuS/shodh-memory"
                target="_blank"
                rel="noopener noreferrer"
                className="px-8 py-4 bg-white dark:bg-slate-900 border border-slate-300 dark:border-slate-700 rounded-lg font-semibold hover:border-primary dark:hover:border-primary transition-colors text-slate-900 dark:text-white"
              >
                View on GitHub
              </a>
            </div>

            <div className="mt-12 grid grid-cols-3 gap-8 max-w-md mx-auto text-sm">
              <a href="https://pypi.org/project/shodh-memory/" target="_blank" rel="noopener noreferrer" className="text-slate-600 dark:text-slate-400 hover:text-primary transition-colors">
                PyPI
              </a>
              <a href="https://www.npmjs.com/package/@shodh/memory-mcp" target="_blank" rel="noopener noreferrer" className="text-slate-600 dark:text-slate-400 hover:text-primary transition-colors">
                npm
              </a>
              <a href="https://crates.io/crates/shodh-memory" target="_blank" rel="noopener noreferrer" className="text-slate-600 dark:text-slate-400 hover:text-primary transition-colors">
                crates.io
              </a>
            </div>
          </motion.div>
        </div>
      </section>
    </main>
  )
}
