'use client'

import { motion } from 'framer-motion'
import Link from 'next/link'
import {
  Brain, Cloud, Lock, Zap, Code, Database, Shield,
  BarChart3, Search, GitBranch, Layers, RefreshCw,
  Globe, Cpu, Server, Laptop, Network, CheckCircle2,
  FileCode, Terminal, Key, Eye, HardDrive, Gauge,
  ArrowRight, ExternalLink, BookOpen
} from 'lucide-react'

export default function Features() {
  const benchmarks = [
    { metric: 'Query Latency (P95)', value: '<10ms', context: 'On 1M+ documents' },
    { metric: 'Citation Accuracy', value: '97%', context: 'With source verification' },
    { metric: 'Indexing Speed', value: '10K docs/min', context: 'With embeddings' },
    { metric: 'Memory Footprint', value: '~200MB', context: 'Base system' },
    { metric: 'Concurrent Users', value: '1000+', context: 'Per node' },
    { metric: 'Vector Dimensions', value: '384-4096', context: 'Configurable' },
  ]

  const architectureComponents = [
    {
      name: 'Document Processor',
      description: 'Handles PDF, DOCX, TXT, Markdown, HTML with structure preservation. Chunking with overlap, metadata extraction.',
      tech: 'Rust + Python bindings',
    },
    {
      name: 'Embedding Engine',
      description: 'Local embedding models (BGE, E5, all-MiniLM) with ONNX Runtime. Optional cloud fallback.',
      tech: 'ONNX + SIMD optimization',
    },
    {
      name: 'Vector Store',
      description: 'HNSW index with configurable M and ef parameters. Supports LMDB, RocksDB, or in-memory.',
      tech: 'Custom Rust implementation',
    },
    {
      name: 'Query Pipeline',
      description: 'Hybrid retrieval combining dense vectors, sparse embeddings (BM25), and reranking.',
      tech: 'Configurable pipeline',
    },
    {
      name: 'Generation Layer',
      description: 'LLM integration with local (Ollama, llama.cpp) or cloud (OpenAI, Anthropic) providers.',
      tech: 'Abstracted LLM interface',
    },
    {
      name: 'Citation Engine',
      description: 'Source attribution with page/line references. Confidence scoring and hallucination detection.',
      tech: 'Probabilistic verification',
    },
  ]

  const apiEndpoints = [
    { method: 'POST', path: '/api/ingest', description: 'Index documents with metadata' },
    { method: 'POST', path: '/api/query', description: 'RAG query with citations' },
    { method: 'GET', path: '/api/search', description: 'Vector similarity search' },
    { method: 'POST', path: '/api/chat', description: 'Conversational RAG' },
    { method: 'GET', path: '/api/documents', description: 'List indexed documents' },
    { method: 'DELETE', path: '/api/documents/:id', description: 'Remove document' },
  ]

  const securityFeatures = [
    {
      icon: Lock,
      name: 'Data Isolation',
      description: 'Multi-tenant architecture with workspace isolation. Data never crosses tenant boundaries.',
    },
    {
      icon: Key,
      name: 'Authentication',
      description: 'API key authentication, JWT tokens, OAuth2 integration. Role-based access control.',
    },
    {
      icon: Shield,
      name: 'Encryption',
      description: 'AES-256 encryption at rest. TLS 1.3 in transit. Optional client-side encryption.',
    },
    {
      icon: Eye,
      name: 'Audit Logging',
      description: 'Complete audit trail of all operations. Exportable logs for compliance.',
    },
    {
      icon: HardDrive,
      name: 'Data Residency',
      description: 'On-premise deployment ensures data never leaves your infrastructure. No cloud dependency.',
    },
    {
      icon: FileCode,
      name: 'Compliance Ready',
      description: 'Designed for GDPR, HIPAA, SOC2 requirements. Self-hosted for maximum control.',
    },
  ]

  const integrations = [
    { name: 'Python SDK', status: 'Available', link: 'https://pypi.org/project/shodh-memory/' },
    { name: 'REST API', status: 'Available', link: null },
    { name: 'MCP Server', status: 'Available', link: 'https://www.npmjs.com/package/@shodh/memory-mcp' },
    { name: 'LangChain', status: 'Coming Soon', link: null },
    { name: 'LlamaIndex', status: 'Coming Soon', link: null },
    { name: 'Ollama', status: 'Supported', link: null },
  ]

  return (
    <main className="min-h-screen bg-gradient-to-b from-white via-slate-50 to-white dark:from-slate-950 dark:via-slate-900 dark:to-slate-950">
      <div className="container mx-auto px-4 py-24">
        {/* Header */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          className="text-center mb-16"
        >
          <div className="inline-block px-4 py-2 bg-primary/10 dark:bg-primary/20 rounded-full text-sm font-semibold text-primary mb-6">
            Technical Deep-Dive
          </div>
          <h1 className="text-5xl md:text-6xl font-bold mb-6 text-slate-900 dark:text-white">
            Architecture &
            <span className="block text-transparent bg-clip-text bg-gradient-to-r from-primary via-secondary to-destructive mt-2">
              Technical Specifications
            </span>
          </h1>
          <p className="text-xl text-slate-600 dark:text-slate-400 max-w-3xl mx-auto">
            Everything your technical team needs to evaluate Shodh.
            Architecture, benchmarks, API specs, and security.
          </p>
        </motion.div>

        {/* Quick Nav */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 0.1 }}
          className="flex flex-wrap justify-center gap-4 mb-20"
        >
          {['Benchmarks', 'Architecture', 'API', 'Security', 'Deployment', 'Integrations'].map((section) => (
            <a
              key={section}
              href={`#${section.toLowerCase()}`}
              className="px-4 py-2 rounded-lg bg-white dark:bg-slate-900 border border-slate-200 dark:border-slate-700 text-sm font-medium text-slate-700 dark:text-slate-300 hover:border-primary dark:hover:border-primary transition-colors"
            >
              {section}
            </a>
          ))}
        </motion.div>

        {/* Benchmarks Section */}
        <section id="benchmarks" className="mb-32 scroll-mt-24">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="text-center mb-12"
          >
            <div className="flex items-center justify-center gap-2 text-primary mb-4">
              <Gauge className="w-5 h-5" />
              <span className="text-sm font-semibold uppercase tracking-wide">Performance</span>
            </div>
            <h2 className="text-3xl md:text-4xl font-bold mb-4 text-slate-900 dark:text-white">
              Benchmarks
            </h2>
            <p className="text-lg text-slate-600 dark:text-slate-400 max-w-2xl mx-auto">
              Real-world performance metrics. Tested on commodity hardware (8 core CPU, 32GB RAM).
            </p>
          </motion.div>

          <div className="grid grid-cols-2 md:grid-cols-3 lg:grid-cols-6 gap-4 max-w-6xl mx-auto">
            {benchmarks.map((benchmark, index) => (
              <motion.div
                key={benchmark.metric}
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                transition={{ delay: index * 0.05 }}
                viewport={{ once: true }}
                className="p-6 rounded-xl border border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900 text-center"
              >
                <div className="text-3xl md:text-4xl font-bold text-primary mb-2">
                  {benchmark.value}
                </div>
                <div className="text-sm font-semibold text-slate-900 dark:text-white mb-1">
                  {benchmark.metric}
                </div>
                <div className="text-xs text-slate-500 dark:text-slate-400">
                  {benchmark.context}
                </div>
              </motion.div>
            ))}
          </div>
        </section>

        {/* Architecture Section */}
        <section id="architecture" className="mb-32 scroll-mt-24">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="text-center mb-12"
          >
            <div className="flex items-center justify-center gap-2 text-secondary mb-4">
              <Layers className="w-5 h-5" />
              <span className="text-sm font-semibold uppercase tracking-wide">System Design</span>
            </div>
            <h2 className="text-3xl md:text-4xl font-bold mb-4 text-slate-900 dark:text-white">
              Architecture Overview
            </h2>
            <p className="text-lg text-slate-600 dark:text-slate-400 max-w-2xl mx-auto">
              Modular components designed for flexibility and performance.
            </p>
          </motion.div>

          {/* Architecture Diagram (Simplified) */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="max-w-4xl mx-auto mb-12 p-8 bg-slate-900 dark:bg-slate-950 rounded-2xl border border-slate-700"
          >
            <div className="font-mono text-sm text-slate-300 overflow-x-auto">
              <pre>{`
┌─────────────────────────────────────────────────────────────────┐
│                        CLIENT APPLICATION                        │
│                  (Python SDK / REST API / MCP)                   │
└─────────────────────────────┬───────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                         QUERY PIPELINE                           │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐   │
│  │   Rewriter   │→ │   Retriever  │→ │   Reranker + Filter  │   │
│  └──────────────┘  └──────────────┘  └──────────────────────┘   │
└─────────────────────────────┬───────────────────────────────────┘
                              │
        ┌─────────────────────┼─────────────────────┐
        ▼                     ▼                     ▼
┌───────────────┐    ┌───────────────┐    ┌───────────────┐
│  Vector Store │    │   BM25 Index  │    │  Graph Store  │
│    (HNSW)     │    │   (Sparse)    │    │  (Optional)   │
└───────────────┘    └───────────────┘    └───────────────┘
        ▲                     ▲
        │                     │
┌───────────────────────────────────────────────────────────────┐
│                      INDEXING PIPELINE                         │
│  ┌──────────┐  ┌───────────┐  ┌──────────┐  ┌──────────────┐  │
│  │  Parser  │→ │  Chunker  │→ │ Embedder │→ │   Storage    │  │
│  └──────────┘  └───────────┘  └──────────┘  └──────────────┘  │
└───────────────────────────────────────────────────────────────┘
              `}</pre>
            </div>
          </motion.div>

          {/* Component Details */}
          <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-6 max-w-6xl mx-auto">
            {architectureComponents.map((component, index) => (
              <motion.div
                key={component.name}
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                transition={{ delay: index * 0.05 }}
                viewport={{ once: true }}
                className="p-6 rounded-xl border border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900"
              >
                <h3 className="text-lg font-semibold mb-2 text-slate-900 dark:text-white">
                  {component.name}
                </h3>
                <p className="text-sm text-slate-600 dark:text-slate-400 mb-3">
                  {component.description}
                </p>
                <div className="inline-block px-2 py-1 bg-slate-100 dark:bg-slate-800 rounded text-xs font-mono text-slate-600 dark:text-slate-400">
                  {component.tech}
                </div>
              </motion.div>
            ))}
          </div>
        </section>

        {/* API Section */}
        <section id="api" className="mb-32 scroll-mt-24">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="text-center mb-12"
          >
            <div className="flex items-center justify-center gap-2 text-destructive mb-4">
              <Terminal className="w-5 h-5" />
              <span className="text-sm font-semibold uppercase tracking-wide">Developer API</span>
            </div>
            <h2 className="text-3xl md:text-4xl font-bold mb-4 text-slate-900 dark:text-white">
              API Reference
            </h2>
            <p className="text-lg text-slate-600 dark:text-slate-400 max-w-2xl mx-auto">
              RESTful API with comprehensive endpoints. Python SDK available.
            </p>
          </motion.div>

          {/* API Endpoints */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="max-w-4xl mx-auto mb-12"
          >
            <div className="rounded-xl border border-slate-200 dark:border-slate-800 overflow-hidden">
              <div className="bg-slate-100 dark:bg-slate-800 px-6 py-3 border-b border-slate-200 dark:border-slate-700">
                <span className="font-semibold text-slate-900 dark:text-white">Core Endpoints</span>
              </div>
              <div className="divide-y divide-slate-200 dark:divide-slate-800">
                {apiEndpoints.map((endpoint) => (
                  <div key={endpoint.path} className="flex items-center gap-4 px-6 py-4 bg-white dark:bg-slate-900">
                    <span className={`px-2 py-1 rounded text-xs font-bold ${
                      endpoint.method === 'GET' ? 'bg-green-100 text-green-700 dark:bg-green-900/30 dark:text-green-400' :
                      endpoint.method === 'POST' ? 'bg-blue-100 text-blue-700 dark:bg-blue-900/30 dark:text-blue-400' :
                      'bg-red-100 text-red-700 dark:bg-red-900/30 dark:text-red-400'
                    }`}>
                      {endpoint.method}
                    </span>
                    <code className="font-mono text-sm text-slate-900 dark:text-white flex-1">
                      {endpoint.path}
                    </code>
                    <span className="text-sm text-slate-500 dark:text-slate-400">
                      {endpoint.description}
                    </span>
                  </div>
                ))}
              </div>
            </div>
          </motion.div>

          {/* Code Example */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="max-w-4xl mx-auto"
          >
            <div className="rounded-xl border border-slate-700 overflow-hidden">
              <div className="bg-slate-800 px-6 py-3 border-b border-slate-700 flex items-center gap-2">
                <Code className="w-4 h-4 text-slate-400" />
                <span className="font-semibold text-slate-200">Python Example</span>
              </div>
              <div className="bg-slate-900 p-6">
                <pre className="text-sm text-slate-300 font-mono overflow-x-auto">
{`from shodh import RAG

# Initialize
rag = RAG(storage_path="./my_docs")

# Index documents
rag.ingest("contracts/", metadata={"type": "legal"})

# Query with citations
result = rag.query(
    "What are the payment terms?",
    filters={"type": "legal"},
    top_k=5
)

print(result.answer)
for citation in result.citations:
    print(f"  - {citation.source}:{citation.page}")`}
                </pre>
              </div>
            </div>
          </motion.div>
        </section>

        {/* Security Section */}
        <section id="security" className="mb-32 scroll-mt-24">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="text-center mb-12"
          >
            <div className="flex items-center justify-center gap-2 text-green-600 dark:text-green-400 mb-4">
              <Shield className="w-5 h-5" />
              <span className="text-sm font-semibold uppercase tracking-wide">Enterprise Security</span>
            </div>
            <h2 className="text-3xl md:text-4xl font-bold mb-4 text-slate-900 dark:text-white">
              Security & Compliance
            </h2>
            <p className="text-lg text-slate-600 dark:text-slate-400 max-w-2xl mx-auto">
              Built for enterprises with strict data governance requirements.
            </p>
          </motion.div>

          <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-6 max-w-6xl mx-auto">
            {securityFeatures.map((feature, index) => (
              <motion.div
                key={feature.name}
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                transition={{ delay: index * 0.05 }}
                viewport={{ once: true }}
                className="p-6 rounded-xl border border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900"
              >
                <div className="w-10 h-10 bg-green-100 dark:bg-green-900/30 rounded-lg flex items-center justify-center mb-4">
                  <feature.icon className="w-5 h-5 text-green-600 dark:text-green-400" />
                </div>
                <h3 className="text-lg font-semibold mb-2 text-slate-900 dark:text-white">
                  {feature.name}
                </h3>
                <p className="text-sm text-slate-600 dark:text-slate-400">
                  {feature.description}
                </p>
              </motion.div>
            ))}
          </div>
        </section>

        {/* Deployment Section */}
        <section id="deployment" className="mb-32 scroll-mt-24">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="text-center mb-12"
          >
            <div className="flex items-center justify-center gap-2 text-purple-600 dark:text-purple-400 mb-4">
              <Server className="w-5 h-5" />
              <span className="text-sm font-semibold uppercase tracking-wide">Infrastructure</span>
            </div>
            <h2 className="text-3xl md:text-4xl font-bold mb-4 text-slate-900 dark:text-white">
              Deployment Options
            </h2>
            <p className="text-lg text-slate-600 dark:text-slate-400 max-w-2xl mx-auto">
              Flexible deployment to match your infrastructure requirements.
            </p>
          </motion.div>

          <div className="grid md:grid-cols-3 gap-8 max-w-6xl mx-auto">
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              className="p-8 rounded-2xl border-2 border-primary/30 bg-gradient-to-br from-primary/5 to-white dark:from-primary/10 dark:to-slate-900"
            >
              <div className="w-14 h-14 bg-primary/10 dark:bg-primary/20 rounded-xl flex items-center justify-center mb-6">
                <Server className="w-7 h-7 text-primary" />
              </div>
              <h3 className="text-xl font-bold mb-3 text-slate-900 dark:text-white">
                On-Premise
              </h3>
              <p className="text-slate-600 dark:text-slate-400 mb-6">
                Full deployment on your infrastructure. Maximum control and data sovereignty.
              </p>
              <div className="space-y-3 text-sm">
                <div className="flex items-start gap-2">
                  <CheckCircle2 className="w-4 h-4 text-primary flex-shrink-0 mt-0.5" />
                  <span className="text-slate-700 dark:text-slate-300">Docker / Kubernetes ready</span>
                </div>
                <div className="flex items-start gap-2">
                  <CheckCircle2 className="w-4 h-4 text-primary flex-shrink-0 mt-0.5" />
                  <span className="text-slate-700 dark:text-slate-300">Air-gapped support</span>
                </div>
                <div className="flex items-start gap-2">
                  <CheckCircle2 className="w-4 h-4 text-primary flex-shrink-0 mt-0.5" />
                  <span className="text-slate-700 dark:text-slate-300">16GB RAM minimum</span>
                </div>
              </div>
            </motion.div>

            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              transition={{ delay: 0.1 }}
              viewport={{ once: true }}
              className="p-8 rounded-2xl border-2 border-secondary/30 bg-gradient-to-br from-secondary/5 to-white dark:from-secondary/10 dark:to-slate-900"
            >
              <div className="w-14 h-14 bg-secondary/10 dark:bg-secondary/20 rounded-xl flex items-center justify-center mb-6">
                <Laptop className="w-7 h-7 text-secondary" />
              </div>
              <h3 className="text-xl font-bold mb-3 text-slate-900 dark:text-white">
                Hybrid
              </h3>
              <p className="text-slate-600 dark:text-slate-400 mb-6">
                Local indexing with optional cloud LLM. Best of both worlds.
              </p>
              <div className="space-y-3 text-sm">
                <div className="flex items-start gap-2">
                  <CheckCircle2 className="w-4 h-4 text-secondary flex-shrink-0 mt-0.5" />
                  <span className="text-slate-700 dark:text-slate-300">Data stays local</span>
                </div>
                <div className="flex items-start gap-2">
                  <CheckCircle2 className="w-4 h-4 text-secondary flex-shrink-0 mt-0.5" />
                  <span className="text-slate-700 dark:text-slate-300">Use any LLM provider</span>
                </div>
                <div className="flex items-start gap-2">
                  <CheckCircle2 className="w-4 h-4 text-secondary flex-shrink-0 mt-0.5" />
                  <span className="text-slate-700 dark:text-slate-300">8GB RAM minimum</span>
                </div>
              </div>
            </motion.div>

            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              transition={{ delay: 0.2 }}
              viewport={{ once: true }}
              className="p-8 rounded-2xl border-2 border-destructive/30 bg-gradient-to-br from-destructive/5 to-white dark:from-destructive/10 dark:to-slate-900"
            >
              <div className="w-14 h-14 bg-destructive/10 dark:bg-destructive/20 rounded-xl flex items-center justify-center mb-6">
                <Network className="w-7 h-7 text-destructive" />
              </div>
              <h3 className="text-xl font-bold mb-3 text-slate-900 dark:text-white">
                Edge / Fleet
              </h3>
              <p className="text-slate-600 dark:text-slate-400 mb-6">
                Distributed deployment for IoT, robotics, and multi-site scenarios.
              </p>
              <div className="space-y-3 text-sm">
                <div className="flex items-start gap-2">
                  <CheckCircle2 className="w-4 h-4 text-destructive flex-shrink-0 mt-0.5" />
                  <span className="text-slate-700 dark:text-slate-300">Zenoh mesh networking</span>
                </div>
                <div className="flex items-start gap-2">
                  <CheckCircle2 className="w-4 h-4 text-destructive flex-shrink-0 mt-0.5" />
                  <span className="text-slate-700 dark:text-slate-300">Offline-first design</span>
                </div>
                <div className="flex items-start gap-2">
                  <CheckCircle2 className="w-4 h-4 text-destructive flex-shrink-0 mt-0.5" />
                  <span className="text-slate-700 dark:text-slate-300">ARM64 support</span>
                </div>
              </div>
            </motion.div>
          </div>
        </section>

        {/* Integrations Section */}
        <section id="integrations" className="mb-24 scroll-mt-24">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            className="text-center mb-12"
          >
            <div className="flex items-center justify-center gap-2 text-orange-600 dark:text-orange-400 mb-4">
              <GitBranch className="w-5 h-5" />
              <span className="text-sm font-semibold uppercase tracking-wide">Ecosystem</span>
            </div>
            <h2 className="text-3xl md:text-4xl font-bold mb-4 text-slate-900 dark:text-white">
              Integrations
            </h2>
            <p className="text-lg text-slate-600 dark:text-slate-400 max-w-2xl mx-auto">
              Connect with your existing tools and workflows.
            </p>
          </motion.div>

          <div className="grid grid-cols-2 md:grid-cols-3 lg:grid-cols-6 gap-4 max-w-5xl mx-auto">
            {integrations.map((integration, index) => (
              <motion.div
                key={integration.name}
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                transition={{ delay: index * 0.05 }}
                viewport={{ once: true }}
                className="p-4 rounded-xl border border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900 text-center"
              >
                <div className="font-semibold text-slate-900 dark:text-white mb-1">
                  {integration.name}
                </div>
                <div className={`text-xs font-medium ${
                  integration.status === 'Available' || integration.status === 'Supported'
                    ? 'text-green-600 dark:text-green-400'
                    : 'text-amber-600 dark:text-amber-400'
                }`}>
                  {integration.status}
                </div>
                {integration.link && (
                  <a
                    href={integration.link}
                    target="_blank"
                    rel="noopener noreferrer"
                    className="inline-flex items-center gap-1 mt-2 text-xs text-primary hover:underline"
                  >
                    View <ExternalLink className="w-3 h-3" />
                  </a>
                )}
              </motion.div>
            ))}
          </div>
        </section>

        {/* CTA */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          viewport={{ once: true }}
          className="text-center p-12 rounded-2xl bg-gradient-to-br from-primary/10 via-secondary/10 to-destructive/10 dark:from-primary/20 dark:via-secondary/20 dark:to-destructive/20"
        >
          <h2 className="text-3xl font-bold mb-4 text-slate-900 dark:text-white">
            Ready to Evaluate?
          </h2>
          <p className="text-xl text-slate-600 dark:text-slate-400 mb-8 max-w-2xl mx-auto">
            Try it yourself in Google Colab, or schedule a technical deep-dive with our team.
          </p>
          <div className="flex flex-col sm:flex-row gap-4 justify-center">
            <Link
              href="/demo"
              className="px-8 py-4 bg-primary hover:bg-primary/90 text-white rounded-lg font-semibold transition-all transform hover:scale-105 flex items-center justify-center gap-2"
            >
              Try Demo
              <ArrowRight className="w-4 h-4" />
            </Link>
            <a
              href="https://github.com/varun29ankuS/shodh-memory"
              target="_blank"
              rel="noopener noreferrer"
              className="px-8 py-4 bg-white dark:bg-slate-900 border border-slate-300 dark:border-slate-700 rounded-lg font-semibold hover:border-primary dark:hover:border-primary transition-colors text-slate-900 dark:text-white flex items-center justify-center gap-2"
            >
              <BookOpen className="w-5 h-5" />
              View Source
            </a>
          </div>
        </motion.div>
      </div>
    </main>
  )
}
