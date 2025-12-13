'use client'

import { motion } from 'framer-motion'
import {
  Play, Code, Calendar, MessageCircle, Github,
  BookOpen, Zap, Brain, ArrowRight, ExternalLink,
  CheckCircle2, FileSearch
} from 'lucide-react'
import Link from 'next/link'

export default function Demo() {
  const colabFeatures = [
    'No installation required',
    'Runs in your browser',
    'Interactive code examples',
    'Try all features instantly',
  ]

  const demoTopics = [
    {
      title: 'See it on YOUR documents',
      description: 'We\'ll show how Shodh works with your actual files',
    },
    {
      title: 'Deployment options',
      description: 'On-premise, hybrid, or cloud - find the right fit',
    },
    {
      title: 'Integration planning',
      description: 'How to connect with your existing systems',
    },
    {
      title: 'Pricing & licensing',
      description: 'Transparent discussion of costs and terms',
    },
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
          <h1 className="text-5xl md:text-6xl font-bold mb-6 text-slate-900 dark:text-white">
            Try Shodh
            <span className="block text-transparent bg-clip-text bg-gradient-to-r from-primary via-secondary to-destructive mt-2">
              Your Way
            </span>
          </h1>
          <p className="text-xl text-slate-600 dark:text-slate-400 max-w-2xl mx-auto">
            Self-serve in Google Colab or book a personalized demo with our team
          </p>
        </motion.div>

        {/* Two Options */}
        <div className="grid md:grid-cols-2 gap-8 max-w-5xl mx-auto mb-20">
          {/* Option 1: Shodh Memory - Self-serve */}
          <motion.div
            initial={{ opacity: 0, x: -20 }}
            animate={{ opacity: 1, x: 0 }}
            transition={{ delay: 0.1 }}
            className="p-8 rounded-2xl border-2 border-primary bg-gradient-to-br from-primary/5 to-primary/10 dark:from-primary/10 dark:to-primary/20"
          >
            <div className="flex items-center gap-3 mb-6">
              <div className="w-12 h-12 bg-primary/20 rounded-xl flex items-center justify-center">
                <Brain className="w-6 h-6 text-primary" />
              </div>
              <div>
                <h3 className="text-lg font-bold text-primary">Shodh Memory</h3>
                <p className="text-xs text-slate-500 dark:text-slate-400">For developers & AI agents</p>
              </div>
            </div>

            <div className="inline-block px-3 py-1 bg-green-500/20 rounded-full text-sm font-semibold text-green-600 dark:text-green-400 mb-4">
              Free & Open Source
            </div>

            <h2 className="text-2xl font-bold mb-3 text-slate-900 dark:text-white">
              Try in Google Colab
            </h2>
            <p className="text-slate-600 dark:text-slate-400 mb-6">
              Interactive notebook with all features. Run code, see results, experiment freely.
            </p>

            <ul className="space-y-2 mb-8">
              {colabFeatures.map((feature) => (
                <li key={feature} className="flex items-center gap-2 text-sm text-slate-700 dark:text-slate-300">
                  <CheckCircle2 className="w-4 h-4 text-primary flex-shrink-0" />
                  {feature}
                </li>
              ))}
            </ul>

            <a
              href="https://colab.research.google.com/github/varun29ankuS/shodh-memory/blob/main/notebooks/shodh_memory_demo.ipynb"
              target="_blank"
              rel="noopener noreferrer"
              className="flex items-center justify-center gap-2 w-full py-4 px-6 rounded-lg font-semibold bg-primary hover:bg-primary/90 text-white transition-all transform hover:scale-105"
            >
              <Play className="w-5 h-5" />
              Open in Colab
              <ExternalLink className="w-4 h-4" />
            </a>

            <p className="text-xs text-slate-500 dark:text-slate-500 mt-4 text-center">
              Free Google account required
            </p>
          </motion.div>

          {/* Option 2: Shodh RAG - Enterprise Demo */}
          <motion.div
            initial={{ opacity: 0, x: 20 }}
            animate={{ opacity: 1, x: 0 }}
            transition={{ delay: 0.2 }}
            className="p-8 rounded-2xl border-2 border-secondary bg-gradient-to-br from-secondary/5 to-secondary/10 dark:from-secondary/10 dark:to-secondary/20"
          >
            <div className="flex items-center gap-3 mb-6">
              <div className="w-12 h-12 bg-secondary/20 rounded-xl flex items-center justify-center">
                <FileSearch className="w-6 h-6 text-secondary" />
              </div>
              <div>
                <h3 className="text-lg font-bold text-secondary">Shodh RAG</h3>
                <p className="text-xs text-slate-500 dark:text-slate-400">For enterprises & teams</p>
              </div>
            </div>

            <div className="inline-block px-3 py-1 bg-secondary/20 rounded-full text-sm font-semibold text-secondary mb-4">
              Free Pilot Available
            </div>

            <h2 className="text-2xl font-bold mb-3 text-slate-900 dark:text-white">
              Book a Demo
            </h2>
            <p className="text-slate-600 dark:text-slate-400 mb-6">
              30-minute call with our team. See Shodh RAG on your documents, discuss your use case.
            </p>

            <ul className="space-y-3 mb-8">
              {demoTopics.map((topic) => (
                <li key={topic.title} className="flex items-start gap-3">
                  <CheckCircle2 className="w-4 h-4 text-secondary flex-shrink-0 mt-1" />
                  <div>
                    <div className="text-sm font-semibold text-slate-900 dark:text-white">{topic.title}</div>
                    <div className="text-xs text-slate-500 dark:text-slate-400">{topic.description}</div>
                  </div>
                </li>
              ))}
            </ul>

            <div className="space-y-3">
              <a
                href="mailto:varun@shodh-rag.com?subject=Shodh RAG Demo Request&body=Hi, I'd like to schedule a demo of Shodh RAG.%0A%0ACompany:%0AUse case:%0APreferred time:"
                className="flex items-center justify-center gap-2 w-full py-4 px-6 rounded-lg font-semibold bg-secondary hover:bg-secondary/90 text-white transition-all transform hover:scale-105"
              >
                <Calendar className="w-5 h-5" />
                Schedule Demo
              </a>

              <a
                href="https://wa.me/919810300618?text=Hi, I'd like to see a demo of Shodh RAG for document search"
                target="_blank"
                rel="noopener noreferrer"
                className="flex items-center justify-center gap-2 w-full py-3 px-6 rounded-lg font-semibold border-2 border-secondary/30 hover:border-secondary text-slate-900 dark:text-white transition-all"
              >
                <MessageCircle className="w-5 h-5" />
                WhatsApp Us
              </a>
            </div>
          </motion.div>
        </div>

        {/* Quick Start Alternative */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          viewport={{ once: true }}
          className="max-w-4xl mx-auto mb-20"
        >
          <div className="text-center mb-8">
            <h2 className="text-2xl font-bold text-slate-900 dark:text-white mb-2">
              Or Run Locally
            </h2>
            <p className="text-slate-600 dark:text-slate-400">
              Install and run on your machine in 30 seconds
            </p>
          </div>

          <div className="p-6 bg-slate-900 dark:bg-slate-950 rounded-xl border border-slate-700">
            <div className="flex items-center gap-2 mb-4 text-slate-400">
              <Code className="w-4 h-4" />
              <span className="text-sm font-semibold">Terminal</span>
            </div>
            <pre className="text-slate-100 font-mono text-sm overflow-x-auto">
{`# Install
pip install shodh-memory

# Quick test
python -c "
from shodh_memory import Memory
m = Memory()
m.remember('Test memory', memory_type='Learning')
print(m.recall('test'))
"`}
            </pre>
          </div>

          <div className="flex justify-center gap-4 mt-6">
            <a
              href="https://pypi.org/project/shodh-memory/"
              target="_blank"
              rel="noopener noreferrer"
              className="flex items-center gap-2 px-4 py-2 rounded-lg bg-slate-100 dark:bg-slate-800 text-slate-700 dark:text-slate-300 hover:bg-slate-200 dark:hover:bg-slate-700 transition-colors text-sm"
            >
              PyPI
              <ExternalLink className="w-3 h-3" />
            </a>
            <a
              href="https://www.npmjs.com/package/@shodh/memory-mcp"
              target="_blank"
              rel="noopener noreferrer"
              className="flex items-center gap-2 px-4 py-2 rounded-lg bg-slate-100 dark:bg-slate-800 text-slate-700 dark:text-slate-300 hover:bg-slate-200 dark:hover:bg-slate-700 transition-colors text-sm"
            >
              npm
              <ExternalLink className="w-3 h-3" />
            </a>
            <a
              href="https://github.com/varun29ankuS/shodh-memory"
              target="_blank"
              rel="noopener noreferrer"
              className="flex items-center gap-2 px-4 py-2 rounded-lg bg-slate-100 dark:bg-slate-800 text-slate-700 dark:text-slate-300 hover:bg-slate-200 dark:hover:bg-slate-700 transition-colors text-sm"
            >
              <Github className="w-4 h-4" />
              GitHub
            </a>
          </div>
        </motion.div>

        {/* What to Expect */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          viewport={{ once: true }}
          className="max-w-4xl mx-auto mb-16"
        >
          <h2 className="text-3xl font-bold mb-8 text-center text-slate-900 dark:text-white">
            What You'll See
          </h2>
          <div className="grid md:grid-cols-3 gap-6">
            {[
              {
                icon: Brain,
                title: 'Memory That Learns',
                description: 'Store memories, watch connections form through Hebbian learning',
              },
              {
                icon: Zap,
                title: 'Instant Recall',
                description: 'Semantic search finds relevant memories in milliseconds',
              },
              {
                icon: BookOpen,
                title: 'Context Bootstrap',
                description: 'Get structured summaries to start any session informed',
              },
            ].map((feature, index) => (
              <motion.div
                key={feature.title}
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                transition={{ delay: index * 0.1 }}
                viewport={{ once: true }}
                className="p-6 rounded-xl border border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900"
              >
                <feature.icon className="w-8 h-8 text-primary mb-4" />
                <h3 className="text-lg font-semibold mb-2 text-slate-900 dark:text-white">
                  {feature.title}
                </h3>
                <p className="text-sm text-slate-600 dark:text-slate-400">{feature.description}</p>
              </motion.div>
            ))}
          </div>
        </motion.div>

        {/* CTA */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          viewport={{ once: true }}
          className="text-center p-12 rounded-2xl bg-gradient-to-br from-primary/10 via-secondary/10 to-destructive/10 dark:from-primary/20 dark:via-secondary/20 dark:to-destructive/20"
        >
          <h2 className="text-3xl font-bold mb-4 text-slate-900 dark:text-white">
            Questions?
          </h2>
          <p className="text-xl text-slate-600 dark:text-slate-400 mb-8">
            Check the docs or reach out directly
          </p>
          <div className="flex flex-col sm:flex-row gap-4 justify-center">
            <Link
              href="/docs"
              className="px-8 py-4 bg-primary hover:bg-primary/90 text-white rounded-lg font-semibold transition-all transform hover:scale-105 flex items-center justify-center gap-2"
            >
              <BookOpen className="w-5 h-5" />
              Documentation
            </Link>
            <a
              href="https://github.com/varun29ankuS/shodh-memory/issues"
              target="_blank"
              rel="noopener noreferrer"
              className="px-8 py-4 bg-white dark:bg-slate-900 border border-slate-300 dark:border-slate-700 rounded-lg font-semibold hover:border-primary dark:hover:border-primary transition-colors text-slate-900 dark:text-white flex items-center justify-center gap-2"
            >
              <Github className="w-5 h-5" />
              GitHub Issues
            </a>
          </div>
        </motion.div>
      </div>
    </main>
  )
}
