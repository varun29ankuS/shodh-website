'use client'

import { motion } from 'framer-motion'
import {
  ArrowRight, Bot, Cpu, Database, GitBranch, Network,
  Radio, Zap, Shield, Brain, Workflow, MessageSquare, Factory, Package, Sprout
} from 'lucide-react'
import Link from 'next/link'

export default function Robotics() {
  const zenohFeatures = [
    {
      icon: Radio,
      title: 'Zero-Copy Pub/Sub',
      description: 'Ultra-low latency message passing for real-time robot control',
    },
    {
      icon: Network,
      title: 'Mesh Networking',
      description: 'Decentralized peer discovery and automatic routing',
    },
    {
      icon: Shield,
      title: 'Built for Edge',
      description: 'Designed for resource-constrained embedded systems',
    },
    {
      icon: GitBranch,
      title: 'Multi-Protocol',
      description: 'TCP, UDP, QUIC, shared memory - choose your transport',
    },
  ]

  const ragRobotCapabilities = [
    {
      icon: Brain,
      title: 'Embodied Intelligence',
      plainEnglish: 'Robots that think before they act',
      description: 'RAG provides context-aware decision making for autonomous robots',
      example: 'Robot queries manuals, SOPs, and environment maps before acting',
    },
    {
      icon: MessageSquare,
      title: 'Natural Language Control',
      plainEnglish: 'Talk to robots like you talk to people',
      description: 'Operators give instructions in plain language, RAG interprets and executes',
      example: '"Navigate to warehouse section B3 and retrieve item X" → RAG finds path, item location, and procedure',
    },
    {
      icon: Database,
      title: 'Offline-First Knowledge',
      plainEnglish: 'Works without internet connectivity',
      description: 'All documentation indexed locally on robot controller - no cloud dependency',
      example: 'Robots work in factories, warehouses, fields without internet',
    },
    {
      icon: Workflow,
      title: 'Multi-Robot Coordination',
      plainEnglish: 'Robots learn from each other instantly',
      description: 'RAG + Zenoh enables fleet-wide knowledge sharing and task coordination',
      example: 'One robot learns new procedure, entire fleet gets updated knowledge via Zenoh mesh',
    },
  ]

  const futureVision = [
    {
      year: '2025-2026',
      title: 'RAG-Enabled Industrial Robots',
      description: 'Factory robots query maintenance manuals, safety protocols, and production schedules in real-time',
      tech: 'Shodh RAG + Zenoh + ROS2',
    },
    {
      year: '2026-2027',
      title: 'Autonomous Warehouse Fleets',
      description: 'Entire warehouse fleets share knowledge mesh - if one robot learns a new route, all robots benefit',
      tech: 'Distributed RAG + Zenoh mesh networking',
    },
    {
      year: '2027-2028',
      title: 'Agricultural Robot Swarms',
      description: 'Field robots coordinate crop management using shared knowledge of soil conditions, weather patterns, and harvest schedules',
      tech: 'Edge RAG + Zenoh + Precision agriculture AI',
    },
    {
      year: '2028+',
      title: 'Fully Embodied AI',
      description: 'Robots with RAG-based reasoning that understand context, follow complex instructions, and learn from experience',
      tech: 'Multimodal RAG + Zenoh + Vision-Language models',
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
              <Bot className="w-4 h-4 text-primary" />
              <span className="text-sm font-semibold text-primary">Local AI for Robotics • No Cloud Required</span>
            </motion.div>

            <h1 className="text-5xl md:text-7xl font-bold mb-6 text-slate-900 dark:text-white">
              Give Your Robots a Brain
              <span className="block text-transparent bg-clip-text bg-gradient-to-r from-primary via-secondary to-destructive mt-2">
                That Works Offline
              </span>
            </h1>

            <p className="text-xl md:text-2xl text-slate-600 dark:text-slate-400 mb-8 max-w-3xl mx-auto">
              Shodh is the local knowledge system that lets robots understand instructions,
              remember procedures, and make decisions—all running on the robot itself.
            </p>

            {/* Technical detail for credibility */}
            <p className="text-base text-slate-500 dark:text-slate-500 mb-12 max-w-2xl mx-auto">
              <strong className="text-primary">Technical:</strong> Shodh RAG + Zenoh = Embodied AI with 50-80ms query latency and real-time fleet coordination
            </p>

            <div className="flex flex-col sm:flex-row gap-4 justify-center mb-12">
              <Link
                href="/demo"
                className="group px-8 py-4 bg-primary hover:bg-primary/90 text-white rounded-lg font-semibold transition-all transform hover:scale-105 flex items-center justify-center gap-2"
              >
                Try RAG Demo
                <ArrowRight className="w-5 h-5 group-hover:translate-x-1 transition-transform" />
              </Link>
              <a
                href="https://zenoh.io"
                target="_blank"
                rel="noopener noreferrer"
                className="px-8 py-4 border-2 border-slate-300 dark:border-slate-700 rounded-lg font-semibold hover:border-primary dark:hover:border-primary transition-colors text-slate-900 dark:text-white"
              >
                Learn About Zenoh
              </a>
            </div>
          </motion.div>
        </div>
      </section>

      {/* Why RAG for Robotics */}
      <section className="relative py-24 bg-gradient-to-b from-white to-slate-50 dark:from-slate-950 dark:to-slate-900">
        <div className="absolute inset-0 bg-grid-slate opacity-30"></div>
        <div className="container mx-auto px-4 relative z-10">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              Why Robots Need RAG (Not Just LLMs)
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400 max-w-3xl mx-auto">
              Robots can't wait for cloud APIs. They need instant access to knowledge, running entirely on-device.
            </p>
          </div>

          {/* Simplified comparison with cards */}
          <div className="max-w-6xl mx-auto mb-16">
            <div className="text-center mb-8">
              <h3 className="text-3xl font-bold mb-3 text-slate-900 dark:text-white">
                Why Robots Can't Wait for Cloud APIs
              </h3>
              <p className="text-lg text-slate-600 dark:text-slate-400">
                Factory robots need instant decisions. Cloud latency = production stoppage.
              </p>
            </div>

            <div className="grid md:grid-cols-2 gap-6 mb-12">
              {/* Cloud Problems Card */}
              <motion.div
                initial={{ opacity: 0, x: -20 }}
                whileInView={{ opacity: 1, x: 0 }}
                viewport={{ once: true }}
                className="p-6 rounded-xl border-2 border-destructive/30 bg-gradient-to-br from-destructive/5 to-white dark:from-destructive/10 dark:to-slate-900"
              >
                <div className="flex items-center gap-3 mb-4">
                  <Shield className="w-7 h-7 text-destructive" />
                  <h4 className="text-xl font-bold text-slate-900 dark:text-white">Cloud LLMs</h4>
                </div>
                <div className="space-y-3 text-sm">
                  <div className="flex items-start gap-2">
                    <span className="text-destructive font-bold text-lg">❌</span>
                    <div>
                      <div className="font-semibold text-slate-900 dark:text-white">500ms+ latency</div>
                      <div className="text-slate-600 dark:text-slate-400">Robot freezes waiting for network</div>
                    </div>
                  </div>
                  <div className="flex items-start gap-2">
                    <span className="text-destructive font-bold text-lg">❌</span>
                    <div>
                      <div className="font-semibold text-slate-900 dark:text-white">₹4.2L/day API costs</div>
                      <div className="text-slate-600 dark:text-slate-400">1000 robots × 1000 queries/day</div>
                    </div>
                  </div>
                  <div className="flex items-start gap-2">
                    <span className="text-destructive font-bold text-lg">❌</span>
                    <div>
                      <div className="font-semibold text-slate-900 dark:text-white">No offline mode</div>
                      <div className="text-slate-600 dark:text-slate-400">WiFi down = robots stop</div>
                    </div>
                  </div>
                </div>
              </motion.div>

              {/* Shodh Benefits Card */}
              <motion.div
                initial={{ opacity: 0, x: 20 }}
                whileInView={{ opacity: 1, x: 0 }}
                viewport={{ once: true }}
                className="p-6 rounded-xl border-2 border-primary/30 bg-gradient-to-br from-primary/5 to-white dark:from-primary/10 dark:to-slate-900"
              >
                <div className="flex items-center gap-3 mb-4">
                  <Brain className="w-7 h-7 text-primary" />
                  <h4 className="text-xl font-bold text-slate-900 dark:text-white">Shodh RAG</h4>
                </div>
                <div className="space-y-3 text-sm">
                  <div className="flex items-start gap-2">
                    <span className="text-primary font-bold text-lg">✓</span>
                    <div>
                      <div className="font-semibold text-slate-900 dark:text-white">50ms response</div>
                      <div className="text-slate-600 dark:text-slate-400">Local index on robot controller</div>
                    </div>
                  </div>
                  <div className="flex items-start gap-2">
                    <span className="text-primary font-bold text-lg">✓</span>
                    <div>
                      <div className="font-semibold text-slate-900 dark:text-white">Zero API costs</div>
                      <div className="text-slate-600 dark:text-slate-400">Run 1M queries for free</div>
                    </div>
                  </div>
                  <div className="flex items-start gap-2">
                    <span className="text-primary font-bold text-lg">✓</span>
                    <div>
                      <div className="font-semibold text-slate-900 dark:text-white">100% offline</div>
                      <div className="text-slate-600 dark:text-slate-400">No internet dependency</div>
                    </div>
                  </div>
                </div>
              </motion.div>
            </div>

            {/* Visual Performance Comparison */}
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              className="p-8 rounded-2xl border border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900"
            >
              <h4 className="text-xl font-bold mb-6 text-center text-slate-900 dark:text-white">
                Response Time: Factory Robot Sees Unknown Object
              </h4>

              <div className="grid md:grid-cols-2 gap-6">
                {/* Cloud Timeline */}
                <div className="p-4 bg-destructive/5 dark:bg-destructive/10 rounded-lg">
                  <div className="font-semibold text-destructive mb-4 flex items-center gap-2 text-sm">
                    <Shield className="w-4 h-4" />
                    Cloud LLM: 700ms
                  </div>
                  <div className="space-y-3 text-xs">
                    <div className="flex items-center gap-2">
                      <div className="w-16 text-slate-500 font-mono">0ms</div>
                      <div className="flex-1 text-slate-700 dark:text-slate-300">Start</div>
                    </div>
                    <div className="flex items-center gap-2">
                      <div className="w-16 text-slate-500 font-mono">200ms</div>
                      <div className="flex-1 text-slate-700 dark:text-slate-300">Network delay</div>
                    </div>
                    <div className="flex items-center gap-2">
                      <div className="w-16 text-slate-500 font-mono">700ms</div>
                      <div className="flex-1 text-slate-700 dark:text-slate-300">GPT-4 response</div>
                    </div>
                    <div className="flex items-center gap-2 pt-2 border-t border-destructive/20">
                      <div className="w-16 text-destructive font-bold font-mono">700ms+</div>
                      <div className="flex-1 text-destructive font-semibold">Line stops</div>
                    </div>
                  </div>
                </div>

                {/* Shodh Timeline */}
                <div className="p-4 bg-primary/5 dark:bg-primary/10 rounded-lg">
                  <div className="font-semibold text-primary mb-4 flex items-center gap-2 text-sm">
                    <Zap className="w-4 h-4" />
                    Shodh RAG: 100ms
                  </div>
                  <div className="space-y-3 text-xs">
                    <div className="flex items-center gap-2">
                      <div className="w-16 text-slate-500 font-mono">0ms</div>
                      <div className="flex-1 text-slate-700 dark:text-slate-300">Start</div>
                    </div>
                    <div className="flex items-center gap-2">
                      <div className="w-16 text-slate-500 font-mono">20ms</div>
                      <div className="flex-1 text-slate-700 dark:text-slate-300">Query local index</div>
                    </div>
                    <div className="flex items-center gap-2">
                      <div className="w-16 text-slate-500 font-mono">50ms</div>
                      <div className="flex-1 text-slate-700 dark:text-slate-300">Find SOPs</div>
                    </div>
                    <div className="flex items-center gap-2 pt-2 border-t border-primary/20">
                      <div className="w-16 text-primary font-bold font-mono">100ms</div>
                      <div className="flex-1 text-primary font-semibold">Decision made</div>
                    </div>
                  </div>
                </div>
              </div>

              {/* Tech Stack Info */}
              <div className="mt-6 p-4 bg-slate-50 dark:bg-slate-800/50 rounded-lg text-center">
                <p className="text-sm text-slate-600 dark:text-slate-400">
                  <strong className="text-slate-900 dark:text-white">Stack:</strong> Shodh (190MB) + llama.cpp (4GB) on robot controller
                </p>
              </div>
            </motion.div>
          </div>
        </div>
      </section>

      {/* Industry Use Cases - NEW SECTION */}
      <section className="relative py-16 bg-white dark:bg-slate-950">
        <div className="container mx-auto px-4">
          <div className="text-center mb-12">
            <h2 className="text-3xl md:text-4xl font-bold mb-4 text-slate-900 dark:text-white">
              Built For Real Robots, Real Industries
            </h2>
            <p className="text-lg text-slate-600 dark:text-slate-400">
              Where local AI makes the difference between working and waiting
            </p>
          </div>

          <div className="grid md:grid-cols-3 gap-6 max-w-5xl mx-auto">
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              className="p-6 rounded-xl border border-slate-200 dark:border-slate-800 bg-slate-50 dark:bg-slate-900 hover:border-primary dark:hover:border-primary transition-all"
            >
              <div className="text-3xl mb-3">🏭</div>
              <h3 className="font-bold text-lg mb-2 text-slate-900 dark:text-white">Manufacturing</h3>
              <p className="text-sm text-slate-600 dark:text-slate-400 mb-3">
                Assembly robots query work instructions, safety procedures, and quality checklists—
                all without internet dependency.
              </p>
              <div className="text-xs text-primary font-semibold">
                → 100ms response vs 700ms cloud delay
              </div>
            </motion.div>

            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              transition={{ delay: 0.1 }}
              viewport={{ once: true }}
              className="p-6 rounded-xl border border-slate-200 dark:border-slate-800 bg-slate-50 dark:bg-slate-900 hover:border-primary dark:hover:border-primary transition-all"
            >
              <div className="text-3xl mb-3">📦</div>
              <h3 className="font-bold text-lg mb-2 text-slate-900 dark:text-white">Warehousing</h3>
              <p className="text-sm text-slate-600 dark:text-slate-400 mb-3">
                Fulfillment robots understand inventory layouts, item locations, and picking
                procedures—even when WiFi drops.
              </p>
              <div className="text-xs text-primary font-semibold">
                → Works offline, zero downtime
              </div>
            </motion.div>

            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              transition={{ delay: 0.2 }}
              viewport={{ once: true }}
              className="p-6 rounded-xl border border-slate-200 dark:border-slate-800 bg-slate-50 dark:bg-slate-900 hover:border-primary dark:hover:border-primary transition-all"
            >
              <div className="text-3xl mb-3">🌾</div>
              <h3 className="font-bold text-lg mb-2 text-slate-900 dark:text-white">Agriculture</h3>
              <p className="text-sm text-slate-600 dark:text-slate-400 mb-3">
                Field robots access crop disease databases, treatment protocols, and weather
                patterns—in areas with no internet.
              </p>
              <div className="text-xs text-primary font-semibold">
                → 100% offline capability
              </div>
            </motion.div>
          </div>
        </div>
      </section>

      {/* Why Zenoh */}
      <section className="relative py-24 bg-gradient-to-b from-slate-50 to-white dark:from-slate-900 dark:to-slate-950">
        <div className="absolute inset-0 bg-grid-slate opacity-30"></div>
        <div className="container mx-auto px-4 relative z-10">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              Why Zenoh for Robotics?
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400 max-w-3xl mx-auto">
              Zenoh is a next-generation pub/sub protocol designed for robotics, IoT, and edge computing.
              It's what makes RAG-powered robots possible.
            </p>
          </div>

          <div className="grid md:grid-cols-2 lg:grid-cols-4 gap-8 max-w-6xl mx-auto mb-16">
            {zenohFeatures.map((feature, index) => (
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
                <p className="text-slate-600 dark:text-slate-400">{feature.description}</p>
              </motion.div>
            ))}
          </div>

          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="max-w-4xl mx-auto p-8 rounded-2xl border-2 border-primary/20 bg-gradient-to-br from-primary/5 to-primary/10 dark:from-primary/10 dark:to-primary/20"
          >
            <h3 className="text-2xl font-bold mb-4 text-slate-900 dark:text-white flex items-center gap-3">
              <Cpu className="w-8 h-8 text-primary" />
              Zenoh vs. Traditional Middleware (ROS, MQTT, DDS)
            </h3>
            <div className="space-y-3 text-slate-700 dark:text-slate-300">
              <p><strong className="text-primary">10-100x lower latency:</strong> Zero-copy shared memory transport</p>
              <p><strong className="text-primary">Mesh networking:</strong> No central broker, automatic peer discovery</p>
              <p><strong className="text-primary">Edge-first:</strong> Runs on microcontrollers with &lt;64KB RAM</p>
              <p><strong className="text-primary">Multi-protocol:</strong> TCP, UDP, QUIC, WebSockets, shared memory - use what you need</p>
              <p><strong className="text-primary">ROS2 compatible:</strong> Drop-in replacement for DDS, works with existing ROS2 code</p>
            </div>
          </motion.div>
        </div>
      </section>

      {/* RAG + Robotics Capabilities */}
      <section className="relative py-24 bg-gradient-to-b from-slate-50 to-white dark:from-slate-900 dark:to-slate-950">
        <div className="absolute inset-0 overflow-hidden">
          <div className="absolute top-1/4 right-0 w-96 h-96 bg-primary/10 rounded-full blur-3xl"></div>
          <div className="absolute bottom-1/4 left-0 w-96 h-96 bg-secondary/10 rounded-full blur-3xl"></div>
        </div>
        <div className="container mx-auto px-4 relative z-10">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              RAG-Powered Robot Capabilities
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400 max-w-3xl mx-auto">
              What happens when you combine Shodh RAG (knowledge) with Zenoh (real-time messaging)?
            </p>
          </div>

          <div className="grid md:grid-cols-2 gap-8 max-w-6xl mx-auto">
            {ragRobotCapabilities.map((capability, index) => (
              <motion.div
                key={capability.title}
                initial={{ opacity: 0, x: index % 2 === 0 ? -20 : 20 }}
                whileInView={{ opacity: 1, x: 0 }}
                transition={{ duration: 0.5 }}
                viewport={{ once: true }}
                className="p-8 rounded-2xl border border-slate-200 dark:border-slate-800 hover:shadow-xl hover:shadow-primary/10 hover:border-primary dark:hover:border-primary transition-all duration-300 bg-white dark:bg-slate-900"
              >
                <div className="flex items-start gap-4 mb-4">
                  <div className="w-12 h-12 bg-primary/10 dark:bg-primary/20 rounded-lg flex items-center justify-center flex-shrink-0">
                    <capability.icon className="w-6 h-6 text-primary" />
                  </div>
                  <div>
                    <h3 className="text-xl font-semibold mb-1 text-slate-900 dark:text-white">{capability.title}</h3>
                    <p className="text-sm text-primary mb-2 font-medium">{capability.plainEnglish}</p>
                    <p className="text-slate-600 dark:text-slate-400 mb-3">{capability.description}</p>
                  </div>
                </div>
                <div className="pl-16">
                  <div className="p-4 bg-slate-50 dark:bg-slate-800 rounded-lg border-l-4 border-primary">
                    <p className="text-sm text-slate-700 dark:text-slate-300">
                      <strong>Example:</strong> {capability.example}
                    </p>
                  </div>
                </div>
              </motion.div>
            ))}
          </div>
        </div>
      </section>

      {/* Architecture Diagram */}
      <section className="relative py-24 bg-gradient-to-b from-white to-slate-50 dark:from-slate-950 dark:to-slate-900">
        <div className="absolute inset-0 bg-grid-slate opacity-30"></div>
        <div className="container mx-auto px-4 relative z-10">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              RAG + Zenoh Architecture
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400">
              How Shodh RAG and Zenoh work together in a robot system
            </p>
          </div>

          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="max-w-5xl mx-auto"
          >
            <div className="p-8 bg-slate-900 dark:bg-slate-950 rounded-2xl border border-slate-700">
              <pre className="text-slate-100 font-mono text-xs md:text-sm overflow-x-auto" style={{ fontFamily: 'Courier New, Consolas, monospace', lineHeight: '1.4', whiteSpace: 'pre' }}>
{`┌─────────────────────────────────────────────────────────────┐
│                    ROBOT CONTROLLER                         │
│                                                             │
│  ┌──────────────────┐        ┌──────────────────┐          │
│  │   SHODH RAG      │        │   ZENOH NODE     │          │
│  │                  │        │                  │          │
│  │  • Local index   │◄──────►│  • Pub/Sub mesh  │          │
│  │  • 1000 docs     │        │  • Zero-copy     │          │
│  │  • LLM (local)   │        │  • Edge routing  │          │
│  └──────────────────┘        └──────────────────┘          │
│           ▲                           ▲                     │
│           │                           │                     │
│           ▼                           ▼                     │
│  ┌─────────────────────────────────────────────┐           │
│  │        ROBOT DECISION ENGINE                │           │
│  │  Query: "Navigate to loading dock B"        │           │
│  │  RAG: Finds map, procedures, safety rules   │           │
│  │  Zenoh: Broadcasts intent to fleet          │           │
│  └─────────────────────────────────────────────┘           │
│                                                             │
└─────────────────────┬───────────────────────────────────────┘
                      │
        ┌─────────────┴─────────────┐
        │                           │
        ▼                           ▼
┌───────────────┐           ┌───────────────┐
│  ROBOT 2      │◄─────────►│  ROBOT 3      │
│  (Zenoh peer) │  Mesh     │  (Zenoh peer) │
│  Shares: Path │  Network  │  Shares: Status│
└───────────────┘           └───────────────┘`}
              </pre>
            </div>

            <div className="mt-8 grid md:grid-cols-3 gap-4">
              <div className="p-4 bg-white dark:bg-slate-900 rounded-lg border border-slate-200 dark:border-slate-800">
                <h4 className="font-semibold text-slate-900 dark:text-white mb-2">1. Query</h4>
                <p className="text-sm text-slate-600 dark:text-slate-400">Robot receives command or senses environment</p>
              </div>
              <div className="p-4 bg-white dark:bg-slate-900 rounded-lg border border-slate-200 dark:border-slate-800">
                <h4 className="font-semibold text-slate-900 dark:text-white mb-2">2. RAG Retrieval</h4>
                <p className="text-sm text-slate-600 dark:text-slate-400">Shodh finds relevant docs, maps, procedures locally</p>
              </div>
              <div className="p-4 bg-white dark:bg-slate-900 rounded-lg border border-slate-200 dark:border-slate-800">
                <h4 className="font-semibold text-slate-900 dark:text-white mb-2">3. Fleet Coordination</h4>
                <p className="text-sm text-slate-600 dark:text-slate-400">Zenoh broadcasts decision to other robots</p>
              </div>
            </div>
          </motion.div>
        </div>
      </section>

      {/* Future Timeline */}
      <section className="relative py-24 bg-gradient-to-b from-slate-50 to-white dark:from-slate-900 dark:to-slate-950">
        <div className="container mx-auto px-4">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              The RAG-Robotics Roadmap
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400">
              Where we are and where we're going
            </p>
          </div>

          <div className="max-w-4xl mx-auto space-y-8">
            {futureVision.map((phase, index) => (
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
                  <div className="inline-block px-3 py-1 bg-primary/10 dark:bg-primary/20 rounded-full mb-3">
                    <span className="text-sm font-semibold text-primary">{phase.year}</span>
                  </div>
                  <h3 className="text-2xl font-bold mb-2 text-slate-900 dark:text-white">{phase.title}</h3>
                  <p className="text-slate-600 dark:text-slate-400 mb-3">{phase.description}</p>
                  <div className="flex items-center gap-2 text-sm text-slate-500 dark:text-slate-500">
                    <Zap className="w-4 h-4" />
                    <span className="font-mono">{phase.tech}</span>
                  </div>
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
              Build RAG-Powered Robots Today
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400 mb-8">
              Shodh RAG is production-ready. Zenoh is production-ready.
              The future of embodied AI starts now.
            </p>
            <div className="flex flex-col sm:flex-row gap-4 justify-center">
              <Link
                href="/demo"
                className="group px-8 py-4 bg-primary hover:bg-primary/90 text-white rounded-lg font-semibold transition-all transform hover:scale-105 flex items-center justify-center gap-2"
              >
                Try Shodh RAG
                <ArrowRight className="w-5 h-5 group-hover:translate-x-1 transition-transform" />
              </Link>
              <a
                href="https://github.com/eclipse-zenoh/zenoh"
                target="_blank"
                rel="noopener noreferrer"
                className="px-8 py-4 bg-white dark:bg-slate-900 border border-slate-300 dark:border-slate-700 rounded-lg font-semibold hover:border-primary dark:hover:border-primary transition-colors text-slate-900 dark:text-white"
              >
                Explore Zenoh on GitHub
              </a>
            </div>
          </motion.div>
        </div>
      </section>
    </main>
  )
}
