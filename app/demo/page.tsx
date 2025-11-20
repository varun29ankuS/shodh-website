'use client'

import { motion } from 'framer-motion'
import { ExternalLink, Play, Code, FileText, MessageSquare } from 'lucide-react'
import Link from 'next/link'

export default function Demo() {
  const demoOptions = [
    {
      icon: Play,
      title: 'Try Shodh RAG',
      description: 'Experience the full-featured desktop application with document intelligence, code analysis, and chat',
      link: 'http://localhost:3001',
      label: 'Launch App',
      highlight: true,
    },
    {
      icon: Code,
      title: 'API Playground',
      description: 'Test the REST and gRPC APIs directly with interactive examples',
      link: '#',
      label: 'Coming Soon',
      highlight: false,
    },
    {
      icon: FileText,
      title: 'Documentation',
      description: 'Comprehensive guides, tutorials, and API reference',
      link: '/docs',
      label: 'Read Docs',
      highlight: false,
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
            Experience SHODH
            <span className="block text-transparent bg-clip-text bg-gradient-to-r from-primary via-secondary to-destructive mt-2">
              First-Hand
            </span>
          </h1>
          <p className="text-xl text-slate-600 dark:text-slate-400 max-w-3xl mx-auto">
            Try our full-featured application or explore the API. No signup required.
          </p>
        </motion.div>

        <div className="grid md:grid-cols-3 gap-8 max-w-6xl mx-auto mb-16">
          {demoOptions.map((option, index) => (
            <motion.div
              key={option.title}
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.5, delay: index * 0.1 }}
              className={`p-8 rounded-2xl border-2 ${
                option.highlight
                  ? 'border-primary bg-gradient-to-br from-primary/5 to-primary/10 dark:from-primary/10 dark:to-primary/20 shadow-xl'
                  : 'border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900'
              }`}
            >
              <div className="w-16 h-16 bg-primary/10 dark:bg-primary/20 rounded-xl flex items-center justify-center mb-6">
                <option.icon className="w-8 h-8 text-primary" />
              </div>

              <h3 className="text-2xl font-bold mb-3 text-slate-900 dark:text-white">
                {option.title}
              </h3>
              <p className="text-slate-600 dark:text-slate-400 mb-6">
                {option.description}
              </p>

              {option.link === '#' ? (
                <button
                  disabled
                  className="w-full py-3 px-6 rounded-lg font-semibold text-center bg-slate-100 dark:bg-slate-800 text-slate-400 cursor-not-allowed"
                >
                  {option.label}
                </button>
              ) : (
                <a
                  href={option.link}
                  target={option.link.startsWith('http') ? '_blank' : undefined}
                  rel={option.link.startsWith('http') ? 'noopener noreferrer' : undefined}
                  className={`flex items-center justify-center gap-2 w-full py-3 px-6 rounded-lg font-semibold text-center transition-all transform hover:scale-105 ${
                    option.highlight
                      ? 'bg-primary hover:bg-primary/90 text-white'
                      : 'bg-slate-100 dark:bg-slate-800 text-slate-900 dark:text-white hover:bg-primary hover:text-white dark:hover:bg-primary'
                  }`}
                >
                  {option.label}
                  {option.link.startsWith('http') && <ExternalLink className="w-4 h-4" />}
                </a>
              )}
            </motion.div>
          ))}
        </div>

        {/* Features Demo */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          viewport={{ once: true }}
          className="max-w-4xl mx-auto mb-16"
        >
          <h2 className="text-3xl font-bold mb-8 text-center text-slate-900 dark:text-white">
            What You'll Experience
          </h2>
          <div className="grid md:grid-cols-2 gap-6">
            {[
              {
                title: 'Document Intelligence',
                description: 'Upload and query documents with precise citations',
              },
              {
                title: 'Code Analysis',
                description: 'Understand codebases with AST-aware search',
              },
              {
                title: 'Knowledge Graphs',
                description: 'Visualize relationships between documents',
              },
              {
                title: 'Multi-Modal Search',
                description: 'Search across text, code, and images',
              },
            ].map((feature, index) => (
              <motion.div
                key={feature.title}
                initial={{ opacity: 0, x: index % 2 === 0 ? -20 : 20 }}
                whileInView={{ opacity: 1, x: 0 }}
                transition={{ delay: index * 0.1 }}
                viewport={{ once: true }}
                className="p-6 rounded-xl border border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900"
              >
                <h3 className="text-lg font-semibold mb-2 text-slate-900 dark:text-white">
                  {feature.title}
                </h3>
                <p className="text-slate-600 dark:text-slate-400">{feature.description}</p>
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
            Ready to Deploy?
          </h2>
          <p className="text-xl text-slate-600 dark:text-slate-400 mb-8">
            Get started with SHODH on your infrastructure today
          </p>
          <div className="flex flex-col sm:flex-row gap-4 justify-center">
            <Link
              href="/pricing"
              className="px-8 py-4 bg-primary hover:bg-primary/90 text-white rounded-lg font-semibold transition-all transform hover:scale-105"
            >
              View Pricing
            </Link>
            <Link
              href="/docs"
              className="px-8 py-4 bg-white dark:bg-slate-900 border border-slate-300 dark:border-slate-700 rounded-lg font-semibold hover:border-primary dark:hover:border-primary transition-colors text-slate-900 dark:text-white"
            >
              Read Docs
            </Link>
          </div>
        </motion.div>
      </div>
    </main>
  )
}
