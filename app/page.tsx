'use client'

import { motion } from 'framer-motion'
import {
  ArrowRight, Search, Shield, Server, IndianRupee,
  FileText, Scale, Building2, HeartPulse, Landmark,
  Clock, Lock, Globe, CheckCircle2, Zap, Database,
  Brain, Code, Languages, MessageCircle
} from 'lucide-react'
import Link from 'next/link'

export default function Home() {
  const painPoints = [
    {
      pain: 'Hours wasted searching through documents',
      solution: 'Find answers in seconds with AI search',
    },
    {
      pain: 'Sensitive data going to US cloud servers',
      solution: 'Everything runs on your servers in India',
    },
    {
      pain: 'Recurring API costs eating into budget',
      solution: 'One-time license, unlimited usage',
    },
    {
      pain: 'No support for Hindi and regional languages',
      solution: 'Built for Indian languages from day one',
    },
  ]

  const capabilities = [
    {
      icon: Search,
      title: 'Ask Questions, Get Answers',
      description: 'Query your documents in plain English or Hindi. Get precise answers with exact page citations.',
    },
    {
      icon: FileText,
      title: 'Works With Your Files',
      description: 'PDFs, Word docs, contracts, emails, spreadsheets. Upload once, search forever.',
    },
    {
      icon: Clock,
      title: 'Answers in Milliseconds',
      description: '50-80ms search speed. No waiting for cloud round-trips.',
    },
    {
      icon: Lock,
      title: 'Your Data Stays Yours',
      description: 'Runs entirely on your infrastructure. Air-gapped deployments supported.',
    },
    {
      icon: Languages,
      title: 'Indian Language Support',
      description: 'Hindi, Tamil, Telugu, Bengali, Marathi and English. More coming.',
    },
    {
      icon: Server,
      title: 'Runs on Existing Hardware',
      description: 'Works on a 16GB laptop. No expensive GPU servers required.',
    },
  ]

  const industries = [
    {
      icon: Scale,
      title: 'Legal & Compliance',
      description: 'Search contracts, case files, regulations. Never miss a clause.',
      examples: ['Contract analysis', 'Due diligence', 'Regulatory compliance'],
    },
    {
      icon: Building2,
      title: 'Banking & Finance',
      description: 'KYC documents, loan files, audit reports. Compliance-ready.',
      examples: ['KYC automation', 'Credit analysis', 'Audit support'],
    },
    {
      icon: HeartPulse,
      title: 'Healthcare',
      description: 'Patient records, research papers, clinical guidelines.',
      examples: ['Medical records search', 'Research synthesis', 'Protocol lookup'],
    },
    {
      icon: Landmark,
      title: 'Government',
      description: 'Policy documents, RTI responses, citizen records.',
      examples: ['Policy search', 'Document digitization', 'Public records'],
    },
  ]

  const trustSignals = [
    {
      icon: Shield,
      title: 'Data Sovereignty',
      description: 'Your documents never leave your servers. No data sent to foreign clouds.',
    },
    {
      icon: Server,
      title: 'On-Premise First',
      description: 'Designed to run locally. Cloud optional, not required.',
    },
    {
      icon: IndianRupee,
      title: '90% Cost Savings',
      description: 'No per-query charges. No monthly subscriptions. One-time license.',
    },
    {
      icon: Globe,
      title: 'Works Offline',
      description: 'No internet required after setup. Perfect for secure environments.',
    },
  ]

  const techStack = [
    { label: 'RAG Architecture', detail: 'Retrieval-Augmented Generation' },
    { label: 'Hybrid Search', detail: 'Vector + keyword (BM25)' },
    { label: 'Local LLM', detail: 'Bring your own or use ours' },
    { label: 'Vector Index', detail: 'Vamana graph (DiskANN)' },
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
              <Shield className="w-4 h-4 text-primary" />
              <span className="text-sm font-semibold text-primary">Your Data. Your Servers. Your Control.</span>
            </motion.div>

            <h1 className="text-5xl md:text-7xl font-bold mb-6 text-slate-900 dark:text-white">
              Search Your Documents
              <span className="block text-transparent bg-clip-text bg-gradient-to-r from-primary via-secondary to-destructive mt-2">
                Like You Search Google
              </span>
            </h1>

            <p className="text-xl md:text-2xl text-slate-600 dark:text-slate-400 mb-8 max-w-3xl mx-auto">
              Ask questions across thousands of documents. Get instant answers with sources.
              Runs on your infrastructure. Data never leaves India.
            </p>

            <div className="flex flex-col sm:flex-row gap-4 justify-center mb-12">
              <Link
                href="/demo"
                className="group px-8 py-4 bg-primary hover:bg-primary/90 text-white rounded-lg font-semibold transition-all transform hover:scale-105 flex items-center justify-center gap-2"
              >
                Try Demo
                <ArrowRight className="w-5 h-5 group-hover:translate-x-1 transition-transform" />
              </Link>
              <a
                href="https://wa.me/919810300618?text=Hi, I'd like to learn more about Shodh"
                target="_blank"
                rel="noopener noreferrer"
                className="px-8 py-4 border-2 border-slate-300 dark:border-slate-700 rounded-lg font-semibold hover:border-primary dark:hover:border-primary transition-colors text-slate-900 dark:text-white flex items-center justify-center gap-2"
              >
                <MessageCircle className="w-5 h-5" />
                Talk to Us
              </a>
            </div>

            {/* Trust badges */}
            <div className="flex flex-wrap justify-center gap-6 text-sm text-slate-600 dark:text-slate-400">
              <div className="flex items-center gap-2">
                <CheckCircle2 className="w-4 h-4 text-primary" />
                <span>100% On-Premise</span>
              </div>
              <div className="flex items-center gap-2">
                <CheckCircle2 className="w-4 h-4 text-primary" />
                <span>Works Offline</span>
              </div>
              <div className="flex items-center gap-2">
                <CheckCircle2 className="w-4 h-4 text-primary" />
                <span>Indian Languages</span>
              </div>
              <div className="flex items-center gap-2">
                <CheckCircle2 className="w-4 h-4 text-primary" />
                <span>No Recurring Costs</span>
              </div>
            </div>
          </motion.div>
        </div>
      </section>

      {/* Pain → Solution */}
      <section className="relative py-24 bg-gradient-to-b from-white to-slate-50 dark:from-slate-950 dark:to-slate-900">
        <div className="container mx-auto px-4">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              Sound Familiar?
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400">
              Problems we solve every day
            </p>
          </div>

          <div className="max-w-4xl mx-auto space-y-6">
            {painPoints.map((item, index) => (
              <motion.div
                key={item.pain}
                initial={{ opacity: 0, x: index % 2 === 0 ? -20 : 20 }}
                whileInView={{ opacity: 1, x: 0 }}
                transition={{ duration: 0.5, delay: index * 0.1 }}
                viewport={{ once: true }}
                className="grid md:grid-cols-2 gap-4 items-center"
              >
                <div className="p-6 rounded-xl bg-destructive/5 dark:bg-destructive/10 border border-destructive/20">
                  <div className="flex items-start gap-3">
                    <span className="text-destructive text-xl">✕</span>
                    <p className="text-slate-700 dark:text-slate-300 font-medium">{item.pain}</p>
                  </div>
                </div>
                <div className="p-6 rounded-xl bg-primary/5 dark:bg-primary/10 border border-primary/20">
                  <div className="flex items-start gap-3">
                    <CheckCircle2 className="w-6 h-6 text-primary flex-shrink-0" />
                    <p className="text-slate-700 dark:text-slate-300 font-medium">{item.solution}</p>
                  </div>
                </div>
              </motion.div>
            ))}
          </div>
        </div>
      </section>

      {/* Two Solutions - Early Product Introduction */}
      <section className="relative py-16 bg-gradient-to-b from-slate-50 to-white dark:from-slate-900 dark:to-slate-950">
        <div className="container mx-auto px-4">
          <div className="text-center mb-10">
            <h2 className="text-3xl md:text-4xl font-bold mb-3 text-slate-900 dark:text-white">
              Two Products. One Mission.
            </h2>
            <p className="text-lg text-slate-600 dark:text-slate-400">
              AI infrastructure that runs on your terms
            </p>
          </div>

          <div className="grid md:grid-cols-2 gap-6 max-w-4xl mx-auto">
            <Link href="/features" className="group">
              <motion.div
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                viewport={{ once: true }}
                className="p-6 rounded-xl border-2 border-primary/20 bg-white dark:bg-slate-900 hover:border-primary transition-all hover:shadow-lg"
              >
                <div className="flex items-center gap-3 mb-3">
                  <Database className="w-8 h-8 text-primary" />
                  <h3 className="text-xl font-bold text-slate-900 dark:text-white">Shodh RAG</h3>
                </div>
                <p className="text-slate-600 dark:text-slate-400 text-sm mb-3">
                  Search thousands of documents with AI. Get answers with citations. For enterprises.
                </p>
                <span className="text-primary font-semibold text-sm inline-flex items-center gap-1 group-hover:gap-2 transition-all">
                  Learn more <ArrowRight className="w-4 h-4" />
                </span>
              </motion.div>
            </Link>

            <Link href="/memory" className="group">
              <motion.div
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                transition={{ delay: 0.1 }}
                viewport={{ once: true }}
                className="p-6 rounded-xl border-2 border-secondary/20 bg-white dark:bg-slate-900 hover:border-secondary transition-all hover:shadow-lg"
              >
                <div className="flex items-center gap-3 mb-3">
                  <Brain className="w-8 h-8 text-secondary" />
                  <h3 className="text-xl font-bold text-slate-900 dark:text-white">Shodh Memory</h3>
                </div>
                <p className="text-slate-600 dark:text-slate-400 text-sm mb-3">
                  Give AI agents persistent memory that learns. Works with Claude, GPT, any agent.
                </p>
                <span className="text-secondary font-semibold text-sm inline-flex items-center gap-1 group-hover:gap-2 transition-all">
                  Learn more <ArrowRight className="w-4 h-4" />
                </span>
              </motion.div>
            </Link>
          </div>
        </div>
      </section>

      {/* Capabilities - Shodh RAG Focus */}
      <section className="relative py-24 bg-gradient-to-b from-white to-slate-50 dark:from-slate-950 dark:to-slate-900">
        <div className="absolute inset-0 bg-grid-slate opacity-30"></div>
        <div className="container mx-auto px-4 relative z-10">
          <div className="text-center mb-16">
            <div className="inline-flex items-center gap-2 px-3 py-1 bg-primary/10 dark:bg-primary/20 rounded-full mb-4">
              <Database className="w-4 h-4 text-primary" />
              <span className="text-sm font-semibold text-primary">Shodh RAG</span>
            </div>
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              Document Intelligence
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400">
              Everything you need to search your documents intelligently
            </p>
          </div>

          <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-8 max-w-6xl mx-auto">
            {capabilities.map((capability, index) => (
              <motion.div
                key={capability.title}
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                transition={{ duration: 0.5, delay: index * 0.1 }}
                viewport={{ once: true }}
                className="p-6 rounded-xl border border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900 hover:border-primary dark:hover:border-primary transition-all hover:shadow-xl hover:shadow-primary/10 hover:-translate-y-1"
              >
                <div className="w-12 h-12 bg-primary/10 dark:bg-primary/20 rounded-lg flex items-center justify-center mb-4">
                  <capability.icon className="w-6 h-6 text-primary" />
                </div>
                <h3 className="text-xl font-semibold mb-2 text-slate-900 dark:text-white">{capability.title}</h3>
                <p className="text-slate-600 dark:text-slate-400">{capability.description}</p>
              </motion.div>
            ))}
          </div>
        </div>
      </section>

      {/* How It Works - Tech Credibility */}
      <section className="relative py-24 bg-gradient-to-b from-white to-slate-50 dark:from-slate-950 dark:to-slate-900">
        <div className="container mx-auto px-4">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              How It Works
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400">
              Enterprise-grade AI, running on your infrastructure
            </p>
          </div>

          <div className="max-w-5xl mx-auto">
            {/* Simple flow */}
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              className="grid md:grid-cols-3 gap-8 mb-16"
            >
              <div className="text-center">
                <div className="w-16 h-16 bg-primary/10 dark:bg-primary/20 rounded-full flex items-center justify-center mx-auto mb-4">
                  <span className="text-2xl font-bold text-primary">1</span>
                </div>
                <h3 className="text-lg font-semibold mb-2 text-slate-900 dark:text-white">Upload Documents</h3>
                <p className="text-slate-600 dark:text-slate-400">PDFs, Word, Excel - we index them all</p>
              </div>
              <div className="text-center">
                <div className="w-16 h-16 bg-primary/10 dark:bg-primary/20 rounded-full flex items-center justify-center mx-auto mb-4">
                  <span className="text-2xl font-bold text-primary">2</span>
                </div>
                <h3 className="text-lg font-semibold mb-2 text-slate-900 dark:text-white">Ask Questions</h3>
                <p className="text-slate-600 dark:text-slate-400">In English, Hindi, or other Indian languages</p>
              </div>
              <div className="text-center">
                <div className="w-16 h-16 bg-primary/10 dark:bg-primary/20 rounded-full flex items-center justify-center mx-auto mb-4">
                  <span className="text-2xl font-bold text-primary">3</span>
                </div>
                <h3 className="text-lg font-semibold mb-2 text-slate-900 dark:text-white">Get Answers</h3>
                <p className="text-slate-600 dark:text-slate-400">With exact citations and sources</p>
              </div>
            </motion.div>

            {/* Tech stack - for evaluators */}
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              className="p-8 rounded-2xl bg-slate-900 dark:bg-slate-950 border border-slate-700"
            >
              <div className="flex items-center gap-2 mb-6">
                <Code className="w-5 h-5 text-slate-400" />
                <span className="text-sm font-semibold text-slate-400">Under the Hood</span>
              </div>

              <div className="grid md:grid-cols-4 gap-6">
                {techStack.map((tech) => (
                  <div key={tech.label} className="text-center">
                    <div className="text-primary font-semibold mb-1">{tech.label}</div>
                    <div className="text-sm text-slate-400">{tech.detail}</div>
                  </div>
                ))}
              </div>

              <div className="mt-6 pt-6 border-t border-slate-700 text-center">
                <Link
                  href="/features"
                  className="text-primary hover:text-primary/80 font-semibold inline-flex items-center gap-2"
                >
                  Technical deep-dive
                  <ArrowRight className="w-4 h-4" />
                </Link>
              </div>
            </motion.div>
          </div>
        </div>
      </section>

      {/* Industries */}
      <section className="relative py-24 bg-gradient-to-b from-slate-50 to-white dark:from-slate-900 dark:to-slate-950">
        <div className="container mx-auto px-4">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              Built for Your Industry
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400">
              Trusted by teams across sectors
            </p>
          </div>

          <div className="grid md:grid-cols-2 lg:grid-cols-4 gap-6 max-w-6xl mx-auto">
            {industries.map((industry, index) => (
              <motion.div
                key={industry.title}
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                transition={{ duration: 0.5, delay: index * 0.1 }}
                viewport={{ once: true }}
                className="p-6 rounded-xl border border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900 hover:border-primary dark:hover:border-primary transition-all"
              >
                <div className="w-12 h-12 bg-primary/10 dark:bg-primary/20 rounded-lg flex items-center justify-center mb-4">
                  <industry.icon className="w-6 h-6 text-primary" />
                </div>
                <h3 className="text-lg font-semibold mb-2 text-slate-900 dark:text-white">{industry.title}</h3>
                <p className="text-sm text-slate-600 dark:text-slate-400 mb-4">{industry.description}</p>
                <ul className="space-y-1">
                  {industry.examples.map((example) => (
                    <li key={example} className="text-xs text-slate-500 dark:text-slate-500 flex items-center gap-2">
                      <CheckCircle2 className="w-3 h-3 text-primary" />
                      {example}
                    </li>
                  ))}
                </ul>
              </motion.div>
            ))}
          </div>
        </div>
      </section>

      {/* Trust Signals */}
      <section className="relative py-24 bg-gradient-to-b from-white to-slate-50 dark:from-slate-950 dark:to-slate-900">
        <div className="container mx-auto px-4">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              Why Teams Trust Shodh
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400">
              Built for Indian enterprises, by Indian engineers
            </p>
          </div>

          <div className="grid md:grid-cols-2 lg:grid-cols-4 gap-8 max-w-6xl mx-auto">
            {trustSignals.map((signal, index) => (
              <motion.div
                key={signal.title}
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                transition={{ duration: 0.5, delay: index * 0.1 }}
                viewport={{ once: true }}
                className="text-center"
              >
                <div className="w-16 h-16 bg-primary/10 dark:bg-primary/20 rounded-full flex items-center justify-center mx-auto mb-4">
                  <signal.icon className="w-8 h-8 text-primary" />
                </div>
                <h3 className="text-lg font-semibold mb-2 text-slate-900 dark:text-white">{signal.title}</h3>
                <p className="text-sm text-slate-600 dark:text-slate-400">{signal.description}</p>
              </motion.div>
            ))}
          </div>
        </div>
      </section>

      {/* Products */}
      <section className="relative py-24 bg-gradient-to-b from-slate-50 to-white dark:from-slate-900 dark:to-slate-950">
        <div className="container mx-auto px-4">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              Our Products
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400">
              Solutions for different needs
            </p>
          </div>

          <div className="grid md:grid-cols-2 gap-8 max-w-4xl mx-auto">
            <motion.div
              initial={{ opacity: 0, x: -20 }}
              whileInView={{ opacity: 1, x: 0 }}
              viewport={{ once: true }}
              className="p-8 rounded-2xl border-2 border-primary/30 bg-gradient-to-br from-primary/5 to-primary/10 dark:from-primary/10 dark:to-primary/20"
            >
              <Database className="w-10 h-10 text-primary mb-4" />
              <h3 className="text-2xl font-bold mb-2 text-slate-900 dark:text-white">Shodh RAG</h3>
              <p className="text-slate-600 dark:text-slate-400 mb-4">
                Document intelligence for enterprises. Search, analyze, and extract insights from your files.
              </p>
              <Link
                href="/features"
                className="text-primary font-semibold inline-flex items-center gap-2 hover:gap-3 transition-all"
              >
                Learn more <ArrowRight className="w-4 h-4" />
              </Link>
            </motion.div>

            <motion.div
              initial={{ opacity: 0, x: 20 }}
              whileInView={{ opacity: 1, x: 0 }}
              viewport={{ once: true }}
              className="p-8 rounded-2xl border-2 border-secondary/30 bg-gradient-to-br from-secondary/5 to-secondary/10 dark:from-secondary/10 dark:to-secondary/20"
            >
              <Brain className="w-10 h-10 text-secondary mb-4" />
              <h3 className="text-2xl font-bold mb-2 text-slate-900 dark:text-white">Shodh Memory</h3>
              <p className="text-slate-600 dark:text-slate-400 mb-4">
                Cognitive memory for AI agents. Give Claude, GPT, or your bots persistent memory that learns.
              </p>
              <Link
                href="/memory"
                className="text-secondary font-semibold inline-flex items-center gap-2 hover:gap-3 transition-all"
              >
                Learn more <ArrowRight className="w-4 h-4" />
              </Link>
            </motion.div>
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
              Ready to Search Smarter?
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400 mb-8">
              See Shodh in action. Try the demo or talk to our team.
            </p>
            <div className="flex flex-col sm:flex-row gap-4 justify-center">
              <Link
                href="/demo"
                className="group px-8 py-4 bg-primary hover:bg-primary/90 text-white rounded-lg font-semibold transition-all transform hover:scale-105 flex items-center justify-center gap-2"
              >
                Try Demo
                <ArrowRight className="w-5 h-5 group-hover:translate-x-1 transition-transform" />
              </Link>
              <a
                href="https://wa.me/919810300618?text=Hi, I'd like to schedule a demo of Shodh"
                target="_blank"
                rel="noopener noreferrer"
                className="px-8 py-4 bg-white dark:bg-slate-900 border border-slate-300 dark:border-slate-700 rounded-lg font-semibold hover:border-primary dark:hover:border-primary transition-colors text-slate-900 dark:text-white flex items-center justify-center gap-2"
              >
                <MessageCircle className="w-5 h-5" />
                WhatsApp Us
              </a>
            </div>
          </motion.div>
        </div>
      </section>
    </main>
  )
}
