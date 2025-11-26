'use client'

import { motion } from 'framer-motion'
import {
  ArrowRight, Brain, Database, Lock, Zap, Clock, Network,
  Shield, HardDrive, Cpu, GitBranch, MessageSquare, Code, Bot, Users, FileText, Search
} from 'lucide-react'
import Link from 'next/link'

export default function Memory() {
  const keyFeatures = [
    {
      icon: Brain,
      title: '3-Tier Memory Hierarchy',
      description: 'Mimics human memory: Working → Session → Long-term with intelligent promotion',
      detail: '<1ms working memory, <10ms session, <100ms long-term retrieval',
    },
    {
      icon: Network,
      title: 'Knowledge Graph Intelligence',
      description: 'Track entities, relationships, and episodes with temporal validity',
      detail: 'Graphiti-inspired architecture for complex knowledge modeling',
    },
    {
      icon: Lock,
      title: '100% Offline & Private',
      description: 'No cloud dependency, GDPR compliant by design, your data stays local',
      detail: 'Multi-tenant isolation with user_id, agent_id, run_id, actor_id',
    },
    {
      icon: Zap,
      title: 'Lightning Fast (Rust)',
      description: '10x faster than Python alternatives, 4MB binary, <100ms startup',
      detail: 'RocksDB storage with LZ4 compression, custom Vamana HNSW index',
    },
    {
      icon: Search,
      title: 'Multi-Modal Retrieval',
      description: '5 search modes: Similarity, Temporal, Causal, Associative, Hybrid',
      detail: 'ONNX Runtime + llama.cpp - bring your own embedding models',
    },
    {
      icon: Shield,
      title: 'Enterprise-Ready',
      description: '28 REST APIs, audit logging, compression, GDPR compliance',
      detail: 'Production-grade with Docker deployment and web dashboard',
    },
  ]

  const useCases = [
    {
      icon: Bot,
      title: 'Autonomous Drones',
      plainEnglish: 'Drones that learn from every flight',
      description: 'Store obstacle maps, terrain data, mission parameters—no internet needed in remote areas',
      example: 'Agricultural drone surveying fields remembers crop patterns, irrigation issues from previous flights. Works in areas with zero connectivity.',
    },
    {
      icon: Shield,
      title: 'Defence & Military Systems',
      plainEnglish: 'Mission-critical memory for secure operations',
      description: 'Air-gapped deployment, classified data stays on-device, complete audit trail for compliance',
      example: 'Reconnaissance drone stores mission data locally. Tactical robots remember terrain, threats, patrol routes—all offline and secure.',
    },
    {
      icon: Cpu,
      title: 'Industrial Robotics',
      plainEnglish: 'Factory robots with persistent knowledge',
      description: 'Assembly procedures, quality checks, maintenance logs—all stored locally on robot controllers',
      example: 'Manufacturing robot recalls "Part X123 requires 45° angle, torque 12Nm" from assembly manual. Warehouse robot optimizes paths based on stored floor layouts.',
    },
    {
      icon: Network,
      title: 'IoT & Smart Devices',
      plainEnglish: 'Edge intelligence without cloud',
      description: 'Smart city sensors, medical devices, surveillance systems—process and remember locally',
      example: 'Smart city traffic sensor builds knowledge graph of congestion patterns. Medical IoT device stores patient context privately without cloud transmission.',
    },
  ]

  const comparisonData = [
    { feature: 'Speed (add memory)', shodh: '<1ms', mem0: '5-10ms', zep: '10-20ms' },
    { feature: 'Speed (semantic search)', shodh: '10-20ms', mem0: '100-200ms', zep: '50-100ms' },
    { feature: 'Deployment', shodh: '100% Edge/Offline', mem0: 'Cloud-only', zep: 'Hybrid' },
    { feature: 'Connectivity', shodh: 'Works Offline', mem0: 'Requires Internet', zep: 'Requires Internet' },
    { feature: 'Memory Hierarchy', shodh: '3-Tier (Unique)', mem0: 'Single-tier', zep: 'Single-tier' },
    { feature: 'Knowledge Graph', shodh: 'Yes (Graphiti)', mem0: 'No', zep: 'Yes (Graphiti)' },
    { feature: 'Language', shodh: 'Rust', mem0: 'Python', zep: 'TypeScript' },
    { feature: 'Binary Size', shodh: '4MB', mem0: '200MB+', zep: '50MB+' },
    { feature: 'Air-Gapped Support', shodh: 'Yes', mem0: 'No', zep: 'No' },
  ]

  const roadmap = [
    {
      year: 'Q1-Q3 2025',
      title: 'Native Python SDK',
      description: 'PyO3 bindings completed - zero IPC overhead, 5-10ms faster than REST API',
      status: 'Completed',
    },
    {
      year: 'Q4 2025',
      title: 'Production Launch',
      description: 'JavaScript/TypeScript SDK, comprehensive documentation, ROS2 integration examples',
      status: 'In Progress',
    },
    {
      year: 'Q1 2026',
      title: 'Advanced Features',
      description: 'GraphQL API, Prometheus metrics, distributed tracing, advanced compression',
      status: 'Planned',
    },
    {
      year: 'Q2 2026',
      title: 'Distributed Mode',
      description: 'Multi-node synchronization for distributed AI agent fleets via Zenoh',
      status: 'Planned',
    },
    {
      year: 'Q3 2026+',
      title: 'Multi-Modal Memory',
      description: 'Image, audio, video embeddings for complete multi-modal memory systems',
      status: 'Vision',
    },
  ]

  return (
    <main className="min-h-screen overflow-hidden">
      {/* Hero */}
      <section className="relative min-h-screen flex items-center justify-center bg-gradient-to-b from-slate-50 via-white to-slate-50 dark:from-slate-950 dark:via-slate-900 dark:to-slate-950 overflow-hidden">
        {/* Animated background */}
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
              <Brain className="w-4 h-4 text-primary" />
              <span className="text-sm font-semibold text-primary">Edge AI Memory • For Robots, Drones & IoT</span>
            </motion.div>

            <h1 className="text-5xl md:text-7xl font-bold mb-6 text-slate-900 dark:text-white">
              AI Memory for the Edge
              <span className="block text-transparent bg-clip-text bg-gradient-to-r from-primary via-secondary to-destructive mt-2">
                Private, Fast, Offline
              </span>
            </h1>

            <p className="text-xl md:text-2xl text-slate-600 dark:text-slate-400 mb-8 max-w-3xl mx-auto">
              Bring persistent memory to robots, drones, IoT devices, and defence systems—running entirely on-device.
              No cloud latency, no connectivity required, no data leaving your edge.
            </p>

            {/* Technical detail for credibility */}
            <p className="text-base text-slate-500 dark:text-slate-500 mb-12 max-w-2xl mx-auto">
              <strong className="text-primary">Edge-Native:</strong> 4MB binary, &lt;100ms retrieval, works offline •
              <strong className="text-primary ml-2">Stack:</strong> Rust + RocksDB + Vamana HNSW + ONNX Runtime + llama.cpp •
              <strong className="text-primary ml-2">Targets:</strong> Drones, Robots, Industrial IoT, Defence
            </p>

            <div className="flex flex-col sm:flex-row gap-4 justify-center mb-12">
              <Link
                href="/getting-started"
                className="group px-8 py-4 bg-primary hover:bg-primary/90 text-white rounded-lg font-semibold transition-all transform hover:scale-105 flex items-center justify-center gap-2"
              >
                Join Beta Program
                <ArrowRight className="w-5 h-5 group-hover:translate-x-1 transition-transform" />
              </Link>
              <Link
                href="/docs"
                className="px-8 py-4 border-2 border-slate-300 dark:border-slate-700 rounded-lg font-semibold hover:border-primary dark:hover:border-primary transition-colors text-slate-900 dark:text-white"
              >
                Documentation
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
                <div className="text-2xl font-bold text-primary">4MB</div>
                <div className="text-sm text-slate-600 dark:text-slate-400">Binary Size</div>
              </motion.div>
              <motion.div
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ delay: 0.4 }}
                className="p-4 bg-white/50 dark:bg-slate-900/50 backdrop-blur-sm rounded-lg border border-slate-200 dark:border-slate-800"
              >
                <div className="text-2xl font-bold text-primary">&lt;100ms</div>
                <div className="text-sm text-slate-600 dark:text-slate-400">Startup Time</div>
              </motion.div>
              <motion.div
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ delay: 0.5 }}
                className="p-4 bg-white/50 dark:bg-slate-900/50 backdrop-blur-sm rounded-lg border border-slate-200 dark:border-slate-800"
              >
                <div className="text-2xl font-bold text-primary">0%</div>
                <div className="text-sm text-slate-600 dark:text-slate-400">Cloud Dependency</div>
              </motion.div>
              <motion.div
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ delay: 0.6 }}
                className="p-4 bg-white/50 dark:bg-slate-900/50 backdrop-blur-sm rounded-lg border border-slate-200 dark:border-slate-800"
              >
                <div className="text-2xl font-bold text-primary">28</div>
                <div className="text-sm text-slate-600 dark:text-slate-400">REST APIs</div>
              </motion.div>
            </div>
          </motion.div>
        </div>
      </section>

      {/* Problem/Solution */}
      <section className="relative py-24 bg-gradient-to-b from-white to-slate-50 dark:from-slate-950 dark:to-slate-900">
        <div className="absolute inset-0 bg-grid-slate opacity-30"></div>
        <div className="container mx-auto px-4 relative z-10">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              Why Edge Devices Need Local Memory
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400 max-w-3xl mx-auto">
              Robots, drones, and IoT devices can't wait for cloud APIs. They need instant, offline intelligence.
            </p>
          </div>

          <div className="grid md:grid-cols-2 gap-8 max-w-5xl mx-auto mb-12">
            {/* Problem Card */}
            <motion.div
              initial={{ opacity: 0, x: -20 }}
              whileInView={{ opacity: 1, x: 0 }}
              viewport={{ once: true }}
              className="p-8 rounded-xl border-2 border-destructive/30 bg-gradient-to-br from-destructive/5 to-white dark:from-destructive/10 dark:to-slate-900"
            >
              <div className="flex items-center gap-3 mb-6">
                <div className="w-12 h-12 bg-destructive/20 rounded-lg flex items-center justify-center">
                  <MessageSquare className="w-6 h-6 text-destructive" />
                </div>
                <h3 className="text-2xl font-bold text-slate-900 dark:text-white">Cloud-Dependent Systems</h3>
              </div>
              <div className="space-y-4 text-sm">
                <div className="flex items-start gap-3">
                  <span className="text-destructive font-bold text-lg">❌</span>
                  <div>
                    <div className="font-semibold text-slate-900 dark:text-white mb-1">Network Latency</div>
                    <div className="text-slate-600 dark:text-slate-400">200-500ms API roundtrip kills real-time decisions</div>
                  </div>
                </div>
                <div className="flex items-start gap-3">
                  <span className="text-destructive font-bold text-lg">❌</span>
                  <div>
                    <div className="font-semibold text-slate-900 dark:text-white mb-1">Connectivity Required</div>
                    <div className="text-slate-600 dark:text-slate-400">Drone in remote field, robot in factory dead zone = system stops</div>
                  </div>
                </div>
                <div className="flex items-start gap-3">
                  <span className="text-destructive font-bold text-lg">❌</span>
                  <div>
                    <div className="font-semibold text-slate-900 dark:text-white mb-1">Data Transmission Risk</div>
                    <div className="text-slate-600 dark:text-slate-400">Sensitive defence/industrial data sent to cloud servers</div>
                  </div>
                </div>
                <div className="flex items-start gap-3">
                  <span className="text-destructive font-bold text-lg">❌</span>
                  <div>
                    <div className="font-semibold text-slate-900 dark:text-white mb-1">Per-API Costs</div>
                    <div className="text-slate-600 dark:text-slate-400">1000 robots × 10K queries/day = ₹4L+/month cloud bills</div>
                  </div>
                </div>
              </div>
            </motion.div>

            {/* Solution Card */}
            <motion.div
              initial={{ opacity: 0, x: 20 }}
              whileInView={{ opacity: 1, x: 0 }}
              viewport={{ once: true }}
              className="p-8 rounded-xl border-2 border-primary/30 bg-gradient-to-br from-primary/5 to-white dark:from-primary/10 dark:to-slate-900"
            >
              <div className="flex items-center gap-3 mb-6">
                <div className="w-12 h-12 bg-primary/20 rounded-lg flex items-center justify-center">
                  <Brain className="w-6 h-6 text-primary" />
                </div>
                <h3 className="text-2xl font-bold text-slate-900 dark:text-white">With Shodh Memory</h3>
              </div>
              <div className="space-y-4 text-sm">
                <div className="flex items-start gap-3">
                  <span className="text-primary font-bold text-lg">✓</span>
                  <div>
                    <div className="font-semibold text-slate-900 dark:text-white mb-1">Sub-100ms Response</div>
                    <div className="text-slate-600 dark:text-slate-400">Local RocksDB + Vamana index = real-time decisions</div>
                  </div>
                </div>
                <div className="flex items-start gap-3">
                  <span className="text-primary font-bold text-lg">✓</span>
                  <div>
                    <div className="font-semibold text-slate-900 dark:text-white mb-1">100% Offline Operation</div>
                    <div className="text-slate-600 dark:text-slate-400">Works in remote fields, factory floors, military zones—no internet needed</div>
                  </div>
                </div>
                <div className="flex items-start gap-3">
                  <span className="text-primary font-bold text-lg">✓</span>
                  <div>
                    <div className="font-semibold text-slate-900 dark:text-white mb-1">Air-Gapped Deployment</div>
                    <div className="text-slate-600 dark:text-slate-400">Classified defence data, sensitive industrial IP—stays on your edge devices</div>
                  </div>
                </div>
                <div className="flex items-start gap-3">
                  <span className="text-primary font-bold text-lg">✓</span>
                  <div>
                    <div className="font-semibold text-slate-900 dark:text-white mb-1">4MB Binary, Low Power</div>
                    <div className="text-slate-600 dark:text-slate-400">Rust efficiency = runs on resource-constrained edge devices</div>
                  </div>
                </div>
              </div>
            </motion.div>
          </div>
        </div>
      </section>

      {/* Key Features */}
      <section className="relative py-24 bg-white dark:bg-slate-950">
        <div className="container mx-auto px-4">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              Enterprise-Grade Memory Features
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400 max-w-3xl mx-auto">
              Built with Rust for performance, designed for production, free for everyone
            </p>
          </div>

          <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-8 max-w-6xl mx-auto">
            {keyFeatures.map((feature, index) => (
              <motion.div
                key={feature.title}
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                transition={{ duration: 0.5, delay: index * 0.1 }}
                viewport={{ once: true }}
                className="group p-6 rounded-xl border border-slate-200 dark:border-slate-800 hover:border-primary dark:hover:border-primary transition-all duration-300 bg-white dark:bg-slate-900 hover:shadow-xl hover:shadow-primary/10 hover:-translate-y-1"
              >
                <div className="w-12 h-12 bg-primary/10 dark:bg-primary/20 rounded-lg flex items-center justify-center mb-4">
                  <feature.icon className="w-6 h-6 text-primary" />
                </div>
                <h3 className="text-xl font-semibold mb-2 text-slate-900 dark:text-white">{feature.title}</h3>
                <p className="text-slate-600 dark:text-slate-400 mb-3">{feature.description}</p>
                <p className="text-xs text-primary font-semibold">{feature.detail}</p>
              </motion.div>
            ))}
          </div>
        </div>
      </section>

      {/* Quick Start Code */}
      <section className="relative py-24 bg-gradient-to-b from-slate-50 to-white dark:from-slate-900 dark:to-slate-950">
        <div className="absolute inset-0 bg-grid-slate opacity-30"></div>
        <div className="container mx-auto px-4 relative z-10">
          <div className="text-center mb-12">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              Get Started in 2 Lines of Code
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400">
              Simple Python API, auto-start server, instant memory
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
              <span className="text-sm font-semibold">Native Python SDK (PyO3) - Zero IPC Overhead</span>
            </div>
            <pre className="text-slate-100 font-mono text-sm overflow-x-auto mb-6">
{`# Install
pip install shodh-memory  # Coming soon

# Import and initialize (native PyO3 bindings)
import shodh_memory

memory = shodh_memory.MemorySystem(storage_path="./my_memory")

# Record experiences
memory.record(
    content="User prefers concise explanations with code examples",
    experience_type="learning",
    entities=["user", "preferences"]
)

memory.record(
    content="John Smith works at OpenAI on GPT-4 development",
    experience_type="discovery",
    entities=["John Smith", "OpenAI", "GPT-4"]
)

# Semantic search (hybrid mode)
results = memory.retrieve(
    query="user preferences for documentation",
    max_results=5,
    mode="hybrid"  # semantic, temporal, or hybrid
)

# Access results
for mem in results:
    print(f"{mem['content']} (importance: {mem['importance']:.2f})")

# Flush to disk before shutdown
memory.flush()`}
            </pre>

            <div className="mt-4 p-4 bg-primary/10 dark:bg-primary/20 rounded-lg border-l-4 border-primary">
              <p className="text-xs text-slate-300 mb-2">
                <strong className="text-primary">⚡ Native Performance:</strong> PyO3 Rust bindings -
                5-10ms faster than REST API, zero IPC overhead, single process deployment.
              </p>
              <p className="text-xs text-slate-300">
                <strong className="text-primary">🔧 Flexible:</strong> Bring your own embedding models via ONNX Runtime.
                Works with llama.cpp for local LLM inference.
              </p>
            </div>

            <div className="grid md:grid-cols-3 gap-4 mt-6">
              <div className="p-4 bg-slate-800 rounded-lg">
                <div className="text-primary font-semibold mb-1 text-sm">Python SDK</div>
                <code className="text-xs text-slate-300">pip install shodh-memory</code>
              </div>
              <div className="p-4 bg-slate-800 rounded-lg">
                <div className="text-primary font-semibold mb-1 text-sm">Docker</div>
                <code className="text-xs text-slate-300">docker-compose up</code>
              </div>
              <div className="p-4 bg-slate-800 rounded-lg">
                <div className="text-primary font-semibold mb-1 text-sm">REST API</div>
                <code className="text-xs text-slate-300">POST /api/record</code>
              </div>
            </div>
          </motion.div>
        </div>
      </section>

      {/* Use Cases */}
      <section className="relative py-24 bg-white dark:bg-slate-950">
        <div className="container mx-auto px-4">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              Built For Real AI Applications
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400">
              Where local memory makes the difference
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
                    <h3 className="text-xl font-semibold mb-1 text-slate-900 dark:text-white">{useCase.title}</h3>
                    <p className="text-sm text-primary mb-2 font-medium">{useCase.plainEnglish}</p>
                    <p className="text-slate-600 dark:text-slate-400 mb-3">{useCase.description}</p>
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

      {/* Architecture Diagram */}
      <section className="relative py-24 bg-gradient-to-b from-slate-50 to-white dark:from-slate-900 dark:to-slate-950">
        <div className="absolute inset-0 bg-grid-slate opacity-30"></div>
        <div className="container mx-auto px-4 relative z-10">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              How Shodh Memory Works
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400">
              3-Tier memory hierarchy + Knowledge graph + Vector search
            </p>
          </div>

          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="max-w-5xl mx-auto"
          >
            <div className="p-8 bg-slate-900 dark:bg-slate-950 rounded-2xl border border-slate-700 mb-8">
              <pre className="text-slate-100 font-mono text-xs md:text-sm overflow-x-auto" style={{ fontFamily: 'Courier New, Consolas, monospace', lineHeight: '1.5', whiteSpace: 'pre' }}>
{`┌──────────────────────────────────────────────────────────────┐
│                    SHODH MEMORY SYSTEM                       │
│                                                              │
│  ┌────────────────────────────────────────────────────────┐  │
│  │           WORKING MEMORY (LRU Cache)                   │  │
│  │  • 100 most recent/frequent memories                   │  │
│  │  • Retrieval: <1ms                                     │  │
│  │  • High importance items (>0.4)                        │  │
│  └────────────────────────────────────────────────────────┘  │
│                          ↓↑                                  │
│  ┌────────────────────────────────────────────────────────┐  │
│  │           SESSION MEMORY (Size-Limited)                │  │
│  │  • 100MB current session context                       │  │
│  │  • Retrieval: <10ms                                    │  │
│  │  • Promotion threshold: importance >0.6                │  │
│  └────────────────────────────────────────────────────────┘  │
│                          ↓↑                                  │
│  ┌────────────────────────────────────────────────────────┐  │
│  │          LONG-TERM MEMORY (RocksDB)                    │  │
│  │  • Unlimited storage with LZ4 compression              │  │
│  │  • Retrieval: <100ms uncompressed, <200ms compressed   │  │
│  │  • Auto-compression after 7+ days                      │  │
│  └────────────────────────────────────────────────────────┘  │
│                                                              │
│  ┌────────────────────────────────────────────────────────┐  │
│  │         KNOWLEDGE GRAPH (Graphiti-Inspired)            │  │
│  │                                                        │  │
│  │  [Entity: John] ──WorksAt──> [Entity: OpenAI]         │  │
│  │        │                            │                  │  │
│  │        └──────Uses──────> [Entity: GPT-4]             │  │
│  │                                                        │  │
│  │  • Entities: Person, Org, Location, Tech, Concept     │  │
│  │  • Relationships: Temporal validity + confidence      │  │
│  │  • Episodes: Time-bounded context windows             │  │
│  └────────────────────────────────────────────────────────┘  │
│                                                              │
│  ┌────────────────────────────────────────────────────────┐  │
│  │         VECTOR INDEX (Custom Vamana HNSW)              │  │
│  │  • ONNX Runtime for embeddings (any model)             │  │
│  │  • Cosine similarity search                            │  │
│  │  • Model-agnostic (384, 512, 768, 1024, 1536 dims)     │  │
│  └────────────────────────────────────────────────────────┘  │
│                                                              │
└──────────────────────────────────────────────────────────────┘
                              ↕
                    ┌──────────────────┐
                    │   28 REST APIs   │
                    │  Python Client   │
                    │   Web Dashboard  │
                    └──────────────────┘`}
              </pre>
            </div>

            <div className="grid md:grid-cols-3 gap-4">
              <div className="p-6 bg-white dark:bg-slate-900 rounded-lg border border-slate-200 dark:border-slate-800">
                <Clock className="w-8 h-8 text-primary mb-3" />
                <h4 className="font-semibold text-slate-900 dark:text-white mb-2">Intelligent Promotion</h4>
                <p className="text-sm text-slate-600 dark:text-slate-400">
                  Memories automatically promoted based on 7-factor importance scoring
                </p>
              </div>
              <div className="p-6 bg-white dark:bg-slate-900 rounded-lg border border-slate-200 dark:border-slate-800">
                <GitBranch className="w-8 h-8 text-primary mb-3" />
                <h4 className="font-semibold text-slate-900 dark:text-white mb-2">Knowledge Graphs</h4>
                <p className="text-sm text-slate-600 dark:text-slate-400">
                  Extract entities and relationships automatically with temporal tracking
                </p>
              </div>
              <div className="p-6 bg-white dark:bg-slate-900 rounded-lg border border-slate-200 dark:border-slate-800">
                <Database className="w-8 h-8 text-primary mb-3" />
                <h4 className="font-semibold text-slate-900 dark:text-white mb-2">Auto-Compression</h4>
                <p className="text-sm text-slate-600 dark:text-slate-400">
                  Old memories compressed automatically with LZ4 (2-5x) or semantic (10-50x)
                </p>
              </div>
            </div>
          </motion.div>
        </div>
      </section>

      {/* Comparison Table */}
      <section className="relative py-24 bg-white dark:bg-slate-950">
        <div className="container mx-auto px-4">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              Why Choose Shodh Memory?
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400">
              Edge-native, offline alternative to cloud-based mem0 ($24M funding) and Zep (Graphiti)
            </p>
          </div>

          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="max-w-5xl mx-auto overflow-x-auto"
          >
            <table className="w-full border-collapse">
              <thead>
                <tr className="border-b-2 border-slate-200 dark:border-slate-800">
                  <th className="text-left p-4 text-slate-900 dark:text-white font-semibold">Feature</th>
                  <th className="text-center p-4 text-primary font-semibold">Shodh Memory</th>
                  <th className="text-center p-4 text-slate-600 dark:text-slate-400 font-semibold">mem0</th>
                  <th className="text-center p-4 text-slate-600 dark:text-slate-400 font-semibold">Zep</th>
                </tr>
              </thead>
              <tbody>
                {comparisonData.map((row, index) => (
                  <tr
                    key={row.feature}
                    className={`border-b border-slate-100 dark:border-slate-800 ${
                      index % 2 === 0 ? 'bg-slate-50 dark:bg-slate-900/50' : ''
                    }`}
                  >
                    <td className="p-4 text-slate-900 dark:text-white font-medium">{row.feature}</td>
                    <td className="p-4 text-center text-primary font-semibold">{row.shodh}</td>
                    <td className="p-4 text-center text-slate-600 dark:text-slate-400">{row.mem0}</td>
                    <td className="p-4 text-center text-slate-600 dark:text-slate-400">{row.zep}</td>
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
              <Shield className="w-8 h-8 text-primary flex-shrink-0 mt-1" />
              <div>
                <h4 className="font-semibold text-slate-900 dark:text-white mb-2">
                  Privacy-First Architecture
                </h4>
                <p className="text-sm text-slate-700 dark:text-slate-300">
                  Unlike cloud-based alternatives, Shodh Memory runs 100% locally. Your data never leaves your machine.
                  Perfect for healthcare, finance, legal, or any privacy-sensitive application. GDPR compliant by design
                  with built-in "Right to be Forgotten" support.
                </p>
              </div>
            </div>
          </motion.div>
        </div>
      </section>

      {/* Roadmap */}
      <section className="relative py-24 bg-gradient-to-b from-slate-50 to-white dark:from-slate-900 dark:to-slate-950">
        <div className="container mx-auto px-4">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              Development Roadmap
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400">
              Where we are and where we're going
            </p>
          </div>

          <div className="max-w-4xl mx-auto space-y-8">
            {roadmap.map((phase, index) => (
              <motion.div
                key={phase.year}
                initial={{ opacity: 0, x: -20 }}
                whileInView={{ opacity: 1, x: 0 }}
                transition={{ duration: 0.5, delay: index * 0.1 }}
                viewport={{ once: true }}
                className="relative pl-8 pb-8 border-l-4 border-primary"
              >
                <div className="absolute -left-3 top-0 w-6 h-6 bg-primary rounded-full border-4 border-white dark:border-slate-950"></div>
                <div className="ml-4">
                  <div className="flex items-center gap-3 mb-3">
                    <span className="inline-block px-3 py-1 bg-primary/10 dark:bg-primary/20 rounded-full text-sm font-semibold text-primary">
                      {phase.year}
                    </span>
                    <span className={`text-xs px-2 py-1 rounded-full ${
                      phase.status === 'Completed'
                        ? 'bg-primary/10 text-primary font-semibold'
                        : phase.status === 'In Progress'
                        ? 'bg-secondary/10 text-secondary'
                        : phase.status === 'Planned'
                        ? 'bg-slate-100 dark:bg-slate-800 text-slate-600 dark:text-slate-400'
                        : 'bg-slate-100 dark:bg-slate-800 text-slate-500 dark:text-slate-500'
                    }`}>
                      {phase.status}
                    </span>
                  </div>
                  <h3 className="text-2xl font-bold mb-2 text-slate-900 dark:text-white">{phase.title}</h3>
                  <p className="text-slate-600 dark:text-slate-400">{phase.description}</p>
                </div>
              </motion.div>
            ))}
          </div>
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
              Bring Intelligence to the Edge
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400 mb-8">
              Deploy Shodh Memory on your robots, drones, and IoT devices. Production-ready, runs completely offline, no cloud required.
            </p>
            <div className="flex flex-col sm:flex-row gap-4 justify-center">
              <Link
                href="/getting-started"
                className="group px-8 py-4 bg-primary hover:bg-primary/90 text-white rounded-lg font-semibold transition-all transform hover:scale-105 flex items-center justify-center gap-2"
              >
                Join Beta Program
                <ArrowRight className="w-5 h-5 group-hover:translate-x-1 transition-transform" />
              </Link>
              <Link
                href="/docs"
                className="px-8 py-4 bg-white dark:bg-slate-900 border border-slate-300 dark:border-slate-700 rounded-lg font-semibold hover:border-primary dark:hover:border-primary transition-colors text-slate-900 dark:text-white"
              >
                Read Documentation
              </Link>
            </div>
          </motion.div>
        </div>
      </section>
    </main>
  )
}
