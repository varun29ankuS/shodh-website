'use client'

import { motion } from 'framer-motion'
import {
  ArrowRight, Check, Zap, Shield, DollarSign, TrendingUp,
  Code, Database, Cloud, Lock, FileText, BookOpen,
  BarChart3, Brain, Sparkles, CheckCircle2
} from 'lucide-react'
import Link from 'next/link'
import InteractiveStats from '@/components/InteractiveStats'
import InteractiveComparison from '@/components/InteractiveComparison'
import VideoHero from '@/components/VideoHero'

export default function Home() {
  const features = [
    {
      icon: Brain,
      title: 'Runs on Existing Hardware',
      description: '500-1000 docs on 16GB laptop. No GPU, no new infrastructure needed',
    },
    {
      icon: Cloud,
      title: 'Index Locally, Search Forever',
      description: 'One-time indexing (overnight). Data never leaves your device',
    },
    {
      icon: DollarSign,
      title: 'LLM Flexibility',
      description: 'Free local models (llama.cpp) OR pay-per-use APIs (GPT-4, Claude)',
    },
    {
      icon: Lock,
      title: '90% Cost Savings',
      description: 'No cloud infrastructure. No vector DB subscriptions. Just software',
    },
    {
      icon: Zap,
      title: '50-80ms Query Speed',
      description: 'Fast local search with Vamana + BM25. No network latency',
    },
    {
      icon: Code,
      title: 'Multi-Language Support',
      description: 'Hindi, Tamil, Telugu, Bengali + English. Built for India',
    },
  ]

  const stats = [
    { value: '1000', label: 'Documents on Laptop' },
    { value: '50-80ms', label: 'Query Latency' },
    { value: '90%', label: 'Cost Savings' },
    { value: '100%', label: 'Data Privacy' },
  ]

  const useCases = [
    {
      title: 'Enterprise Search',
      description: 'Search across all company documents with accurate citations',
      icon: FileText,
    },
    {
      title: 'Code Intelligence',
      description: 'Understand and query large codebases with ease',
      icon: Code,
    },
    {
      title: 'Knowledge Management',
      description: 'Build intelligent knowledge bases that actually work',
      icon: BookOpen,
    },
    {
      title: 'Research & Analysis',
      description: 'Extract insights from vast document collections',
      icon: BarChart3,
    },
  ]

  const comparison = {
    shodh: [
      'One-time license fee',
      'Runs on your infrastructure',
      '97% citation accuracy',
      'No data sent externally',
      'Unlimited queries',
      'Full source code access',
    ],
    cloudApis: [
      'Pay per million tokens',
      'Cloud-only (data leaves your servers)',
      'Limited citation tracking',
      'Vendor lock-in',
      'Usage-based billing',
      'Black box systems',
    ],
  }

  return (
    <main className="min-h-screen overflow-hidden">
      {/* Hero */}
      <section className="relative min-h-screen flex items-center justify-center bg-gradient-to-b from-slate-50 via-white to-slate-50 dark:from-slate-950 dark:via-slate-900 dark:to-slate-950 overflow-hidden">
        {/* Enhanced animated background */}
        <div className="absolute inset-0 overflow-hidden">
          {/* Larger animated orbs */}
          <div className="absolute -top-40 -left-40 w-96 h-96 bg-primary/30 rounded-full blur-3xl animate-pulse"></div>
          <div className="absolute top-1/3 -right-40 w-[600px] h-[600px] bg-secondary/25 rounded-full blur-3xl animate-pulse" style={{ animationDelay: '1s' }}></div>
          <div className="absolute -bottom-40 left-1/3 w-[500px] h-[500px] bg-destructive/30 rounded-full blur-3xl animate-pulse" style={{ animationDelay: '2s' }}></div>

          {/* Moving gradient orbs */}
          <div className="absolute top-1/4 left-1/4 w-72 h-72 bg-gradient-to-br from-primary/20 to-transparent rounded-full blur-2xl animate-float"></div>
          <div className="absolute bottom-1/4 right-1/4 w-80 h-80 bg-gradient-to-br from-secondary/20 to-transparent rounded-full blur-2xl animate-float-delayed"></div>
        </div>
        <div className="absolute inset-0 bg-grid-slate opacity-40" />
        <div className="absolute inset-0 gradient-mesh"></div>

        {/* Animated lines/particles effect */}
        <div className="absolute inset-0 opacity-30">
          <div className="absolute top-0 left-1/4 w-px h-full bg-gradient-to-b from-transparent via-primary/50 to-transparent animate-pulse"></div>
          <div className="absolute top-0 left-2/4 w-px h-full bg-gradient-to-b from-transparent via-secondary/50 to-transparent animate-pulse" style={{ animationDelay: '0.5s' }}></div>
          <div className="absolute top-0 left-3/4 w-px h-full bg-gradient-to-b from-transparent via-destructive/50 to-transparent animate-pulse" style={{ animationDelay: '1s' }}></div>
        </div>
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
              <Sparkles className="w-4 h-4 text-primary" />
              <span className="text-sm font-semibold text-primary">Enterprise RAG • Run Locally • LLM Flexible</span>
            </motion.div>

            <h1 className="text-5xl md:text-7xl font-bold mb-6 text-slate-900 dark:text-white">
              Production RAG on
              <span className="block text-transparent bg-clip-text bg-gradient-to-r from-primary via-secondary to-destructive mt-2">
                Your Existing Laptop
              </span>
            </h1>

            <p className="text-xl md:text-2xl text-slate-600 dark:text-slate-400 mb-12 max-w-3xl mx-auto">
              Index locally once (overnight), search privately forever.
              Choose your LLM: free local models or pay-per-use APIs.
              90% cost savings. Zero new infrastructure.
            </p>

            <div className="flex flex-col sm:flex-row gap-4 justify-center mb-12">
              <Link
                href="/demo"
                className="group px-8 py-4 bg-primary hover:bg-primary/90 text-white rounded-lg font-semibold transition-all transform hover:scale-105 flex items-center justify-center gap-2"
              >
                Try Demo
                <ArrowRight className="w-5 h-5 group-hover:translate-x-1 transition-transform" />
              </Link>
              <Link
                href="/pricing"
                className="px-8 py-4 border-2 border-slate-300 dark:border-slate-700 rounded-lg font-semibold hover:border-primary dark:hover:border-primary transition-colors text-slate-900 dark:text-white"
              >
                View Pricing
              </Link>
            </div>

            {/* Stats */}
            <InteractiveStats stats={stats} />
          </motion.div>
        </div>
      </section>

      {/* Features */}
      <section className="relative py-24 bg-gradient-to-b from-white to-slate-50 dark:from-slate-950 dark:to-slate-900">
        <div className="absolute inset-0 bg-grid-slate opacity-30"></div>
        <div className="container mx-auto px-4">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              Why Teams Choose SHODH
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400 max-w-2xl mx-auto">
              Everything you need for production-grade RAG, without the complexity
            </p>
          </div>

          <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-8 max-w-6xl mx-auto">
            {features.map((feature, index) => (
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
        </div>
      </section>

      {/* Comparison */}
      <section className="relative py-24 bg-gradient-to-b from-slate-50 to-white dark:from-slate-900 dark:to-slate-950">
        <div className="absolute inset-0 overflow-hidden">
          <div className="absolute top-1/4 right-0 w-96 h-96 bg-primary/10 rounded-full blur-3xl"></div>
          <div className="absolute bottom-1/4 left-0 w-96 h-96 bg-secondary/10 rounded-full blur-3xl"></div>
        </div>
        <div className="container mx-auto px-4 relative z-10">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              On-Premise vs. Cloud APIs
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400">
              Own your infrastructure, control your costs
            </p>
          </div>

          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.5 }}
            viewport={{ once: true }}
            className="max-w-5xl mx-auto"
          >
            <InteractiveComparison />
          </motion.div>
        </div>
      </section>

      {/* Video Section */}
      <VideoHero />

      {/* Use Cases */}
      <section className="relative py-24 bg-gradient-to-b from-slate-50 to-white dark:from-slate-900 dark:to-slate-950">
        <div className="absolute inset-0 bg-grid-slate opacity-20"></div>
        <div className="container mx-auto px-4 relative z-10">
          <div className="text-center mb-16">
            <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
              Built for Every Use Case
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400">
              From enterprise search to code intelligence
            </p>
          </div>

          <div className="grid md:grid-cols-2 lg:grid-cols-4 gap-6 max-w-6xl mx-auto">
            {useCases.map((useCase, index) => (
              <motion.div
                key={useCase.title}
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                transition={{ duration: 0.5, delay: index * 0.1 }}
                viewport={{ once: true }}
                className="group p-6 rounded-xl border border-slate-200 dark:border-slate-800 hover:shadow-xl hover:shadow-primary/10 hover:border-primary dark:hover:border-primary transition-all duration-300 bg-white dark:bg-slate-900 hover:-translate-y-1"
              >
                <div className="w-10 h-10 bg-primary/10 dark:bg-primary/20 rounded-lg flex items-center justify-center mb-4">
                  <useCase.icon className="w-5 h-5 text-primary" />
                </div>
                <h3 className="text-lg font-semibold mb-2 text-slate-900 dark:text-white">{useCase.title}</h3>
                <p className="text-sm text-slate-600 dark:text-slate-400">{useCase.description}</p>
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
              Ready to Take Control of Your RAG?
            </h2>
            <p className="text-xl text-slate-600 dark:text-slate-400 mb-8">
              Stop sending your data to third parties. Start with SHODH today.
            </p>
            <div className="flex flex-col sm:flex-row gap-4 justify-center">
              <Link
                href="/demo"
                className="group px-8 py-4 bg-primary hover:bg-primary/90 text-white rounded-lg font-semibold transition-all transform hover:scale-105 flex items-center justify-center gap-2"
              >
                Try Demo Now
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
