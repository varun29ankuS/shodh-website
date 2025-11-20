'use client'

import { motion } from 'framer-motion'
import {
  Brain, Cloud, DollarSign, Lock, Zap, Code, Database, Shield,
  BarChart3, FileText, Search, GitBranch, MessageSquare, Workflow,
  Layers, RefreshCw, Globe, Cpu, Server, Laptop, Network
} from 'lucide-react'

export default function Features() {
  const featureCategories = [
    {
      title: 'Core RAG Capabilities',
      description: 'Industry-leading retrieval and generation',
      features: [
        {
          icon: Brain,
          name: '97% Citation Accuracy',
          description: 'Verifiable source attribution with exact page/line references',
        },
        {
          icon: Search,
          name: 'SIMD-Optimized Search',
          description: 'Sub-millisecond vector similarity search with hardware acceleration',
        },
        {
          icon: Database,
          name: 'Multiple Vector Stores',
          description: 'Support for LMDB, RocksDB, and in-memory backends',
        },
        {
          icon: Layers,
          name: 'Hybrid Search',
          description: 'Combine dense vectors, sparse embeddings, and full-text search',
        },
      ],
    },
    {
      title: 'Enterprise Features',
      description: 'Built for production at scale',
      features: [
        {
          icon: Cloud,
          name: 'On-Premise Deployment',
          description: 'Run entirely on your infrastructure. Zero data egress.',
        },
        {
          icon: Lock,
          name: 'Data Privacy',
          description: 'Your data never leaves your servers. Full compliance control.',
        },
        {
          icon: Shield,
          name: 'Enterprise Security',
          description: 'Role-based access, audit logs, and encryption at rest',
        },
        {
          icon: BarChart3,
          name: 'Monitoring & Metrics',
          description: 'Built-in observability with Prometheus/Grafana support',
        },
      ],
    },
    {
      title: 'Developer Experience',
      description: 'Tools developers love',
      features: [
        {
          icon: Code,
          name: 'Code-Aware Intelligence',
          description: 'Understands code structure, AST, and relationships',
        },
        {
          icon: GitBranch,
          name: 'Version Control Integration',
          description: 'Track document versions and changes over time',
        },
        {
          icon: MessageSquare,
          name: 'REST & gRPC APIs',
          description: 'Multiple integration options with comprehensive docs',
        },
        {
          icon: Workflow,
          name: 'Workflow Automation',
          description: 'Build complex RAG pipelines with our agent system',
        },
      ],
    },
    {
      title: 'Performance & Scale',
      description: 'Designed for high-throughput workloads',
      features: [
        {
          icon: Zap,
          name: 'Lightning Fast',
          description: '<10ms query latency with optimized indexing',
        },
        {
          icon: Cpu,
          name: 'Resource Efficient',
          description: 'Run on commodity hardware. Scale horizontally.',
        },
        {
          icon: RefreshCw,
          name: 'Real-Time Updates',
          description: 'Incremental indexing with zero downtime',
        },
        {
          icon: Globe,
          name: 'Multi-Tenant',
          description: 'Isolated workspaces for different teams/projects',
        },
      ],
    },
  ]

  return (
    <main className="min-h-screen bg-gradient-to-b from-white via-slate-50 to-white dark:from-slate-950 dark:via-slate-900 dark:to-slate-950">
      <div className="container mx-auto px-4 py-24">
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          className="text-center mb-16"
        >
          <h1 className="text-5xl md:text-6xl font-bold mb-6 text-slate-900 dark:text-white">
            Everything You Need for
            <span className="block text-transparent bg-clip-text bg-gradient-to-r from-primary via-secondary to-destructive mt-2">
              Production-Grade RAG
            </span>
          </h1>
          <p className="text-xl text-slate-600 dark:text-slate-400 max-w-3xl mx-auto">
            SHODH combines cutting-edge research with battle-tested engineering to deliver the most accurate and reliable RAG system available.
          </p>
        </motion.div>

        <div className="space-y-24">
          {featureCategories.map((category, categoryIndex) => (
            <motion.div
              key={category.title}
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.5 }}
              viewport={{ once: true }}
              className="max-w-6xl mx-auto"
            >
              <div className="text-center mb-12">
                <h2 className="text-3xl md:text-4xl font-bold mb-4 text-slate-900 dark:text-white">
                  {category.title}
                </h2>
                <p className="text-lg text-slate-600 dark:text-slate-400">
                  {category.description}
                </p>
              </div>

              <div className="grid md:grid-cols-2 gap-6">
                {category.features.map((feature, featureIndex) => (
                  <motion.div
                    key={feature.name}
                    initial={{ opacity: 0, y: 20 }}
                    whileInView={{ opacity: 1, y: 0 }}
                    transition={{ duration: 0.5, delay: featureIndex * 0.1 }}
                    viewport={{ once: true }}
                    className="p-6 rounded-xl border border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900 hover:border-primary dark:hover:border-primary transition-all hover:shadow-lg"
                  >
                    <div className="flex items-start gap-4">
                      <div className="w-12 h-12 bg-primary/10 dark:bg-primary/20 rounded-lg flex items-center justify-center flex-shrink-0">
                        <feature.icon className="w-6 h-6 text-primary" />
                      </div>
                      <div>
                        <h3 className="text-xl font-semibold mb-2 text-slate-900 dark:text-white">
                          {feature.name}
                        </h3>
                        <p className="text-slate-600 dark:text-slate-400">
                          {feature.description}
                        </p>
                      </div>
                    </div>
                  </motion.div>
                ))}
              </div>
            </motion.div>
          ))}
        </div>

        {/* Deployment Architectures - NEW SECTION */}
        <div className="mt-32">
          <div className="text-center mb-16">
            <h2 className="text-3xl md:text-4xl font-bold mb-4 text-slate-900 dark:text-white">
              Flexible Deployment Options
            </h2>
            <p className="text-lg text-slate-600 dark:text-slate-400 max-w-2xl mx-auto">
              Deploy Shodh your way - full-stack local, hybrid architecture, or distributed fleet
            </p>
          </div>

          <div className="grid md:grid-cols-3 gap-8 max-w-6xl mx-auto">
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              className="p-8 rounded-2xl border-2 border-primary/30 bg-gradient-to-br from-primary/5 to-white dark:from-primary/10 dark:to-slate-900 hover:border-primary transition-all"
            >
              <div className="w-16 h-16 bg-primary/10 dark:bg-primary/20 rounded-xl flex items-center justify-center mb-6">
                <Server className="w-8 h-8 text-primary" />
              </div>
              <h3 className="text-2xl font-bold mb-3 text-slate-900 dark:text-white">
                Full-Stack Local
              </h3>
              <p className="text-slate-600 dark:text-slate-400 mb-4">
                Complete RAG pipeline with local LLM. Maximum privacy, zero cloud dependency.
              </p>
              <div className="space-y-2 text-sm">
                <div className="flex items-center gap-2">
                  <div className="w-1.5 h-1.5 bg-primary rounded-full"></div>
                  <span className="text-slate-700 dark:text-slate-300">Recommended: 16GB+ RAM</span>
                </div>
                <div className="flex items-center gap-2">
                  <div className="w-1.5 h-1.5 bg-primary rounded-full"></div>
                  <span className="text-slate-700 dark:text-slate-300">4+ CPU cores</span>
                </div>
                <div className="flex items-center gap-2">
                  <div className="w-1.5 h-1.5 bg-primary rounded-full"></div>
                  <span className="text-slate-700 dark:text-slate-300">Use case: Enterprise, data sovereignty</span>
                </div>
              </div>
            </motion.div>

            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              transition={{ delay: 0.1 }}
              viewport={{ once: true }}
              className="p-8 rounded-2xl border-2 border-secondary/30 bg-gradient-to-br from-secondary/5 to-white dark:from-secondary/10 dark:to-slate-900 hover:border-secondary transition-all"
            >
              <div className="w-16 h-16 bg-secondary/10 dark:bg-secondary/20 rounded-xl flex items-center justify-center mb-6">
                <Laptop className="w-8 h-8 text-secondary" />
              </div>
              <h3 className="text-2xl font-bold mb-3 text-slate-900 dark:text-white">
                Hybrid Architecture
              </h3>
              <p className="text-slate-600 dark:text-slate-400 mb-4">
                Local retrieval with cloud/API LLM integration. Flexible and cost-effective.
              </p>
              <div className="space-y-2 text-sm">
                <div className="flex items-center gap-2">
                  <div className="w-1.5 h-1.5 bg-secondary rounded-full"></div>
                  <span className="text-slate-700 dark:text-slate-300">Works on edge devices</span>
                </div>
                <div className="flex items-center gap-2">
                  <div className="w-1.5 h-1.5 bg-secondary rounded-full"></div>
                  <span className="text-slate-700 dark:text-slate-300">Cloud LLM when needed</span>
                </div>
                <div className="flex items-center gap-2">
                  <div className="w-1.5 h-1.5 bg-secondary rounded-full"></div>
                  <span className="text-slate-700 dark:text-slate-300">Use case: Drones, IoT, mobile</span>
                </div>
              </div>
            </motion.div>

            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              transition={{ delay: 0.2 }}
              viewport={{ once: true }}
              className="p-8 rounded-2xl border-2 border-destructive/30 bg-gradient-to-br from-destructive/5 to-white dark:from-destructive/10 dark:to-slate-900 hover:border-destructive transition-all"
            >
              <div className="w-16 h-16 bg-destructive/10 dark:bg-destructive/20 rounded-xl flex items-center justify-center mb-6">
                <Network className="w-8 h-8 text-destructive" />
              </div>
              <h3 className="text-2xl font-bold mb-3 text-slate-900 dark:text-white">
                Distributed Fleet
              </h3>
              <p className="text-slate-600 dark:text-slate-400 mb-4">
                Centralized RAG with Zenoh mesh networking. Fleet-wide knowledge sharing.
              </p>
              <div className="space-y-2 text-sm">
                <div className="flex items-center gap-2">
                  <div className="w-1.5 h-1.5 bg-destructive rounded-full"></div>
                  <span className="text-slate-700 dark:text-slate-300">Ground station: 16GB+ RAM</span>
                </div>
                <div className="flex items-center gap-2">
                  <div className="w-1.5 h-1.5 bg-destructive rounded-full"></div>
                  <span className="text-slate-700 dark:text-slate-300">Edge nodes: Lightweight clients</span>
                </div>
                <div className="flex items-center gap-2">
                  <div className="w-1.5 h-1.5 bg-destructive rounded-full"></div>
                  <span className="text-slate-700 dark:text-slate-300">Use case: Multi-robot fleets</span>
                </div>
              </div>
            </motion.div>
          </div>
        </div>

        {/* CTA */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          viewport={{ once: true }}
          className="text-center mt-24 p-12 rounded-2xl bg-gradient-to-br from-primary/10 via-secondary/10 to-destructive/10 dark:from-primary/20 dark:via-secondary/20 dark:to-destructive/20"
        >
          <h2 className="text-3xl font-bold mb-4 text-slate-900 dark:text-white">
            See SHODH in Action
          </h2>
          <p className="text-xl text-slate-600 dark:text-slate-400 mb-8">
            Try our interactive demo or schedule a personalized walkthrough
          </p>
          <div className="flex flex-col sm:flex-row gap-4 justify-center">
            <a
              href="/demo"
              className="px-8 py-4 bg-primary hover:bg-primary/90 text-white rounded-lg font-semibold transition-all transform hover:scale-105"
            >
              Try Demo
            </a>
            <a
              href="/pricing"
              className="px-8 py-4 bg-white dark:bg-slate-900 border border-slate-300 dark:border-slate-700 rounded-lg font-semibold hover:border-primary dark:hover:border-primary transition-colors text-slate-900 dark:text-white"
            >
              View Pricing
            </a>
          </div>
        </motion.div>
      </div>
    </main>
  )
}
