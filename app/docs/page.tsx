'use client'

import { motion } from 'framer-motion'
import {
  Book, Code, Terminal, Rocket, FileText, Settings,
  Database, Cloud, Lock, Zap, ArrowRight
} from 'lucide-react'
import Link from 'next/link'

export default function Docs() {
  const docSections = [
    {
      title: 'Getting Started',
      icon: Rocket,
      links: [
        { name: 'Quick Start', href: '#' },
        { name: 'Installation', href: '#' },
        { name: 'Configuration', href: '#' },
        { name: 'First Query', href: '#' },
      ],
    },
    {
      title: 'Core Concepts',
      icon: Book,
      links: [
        { name: 'How RAG Works', href: '#' },
        { name: 'Vector Search', href: '#' },
        { name: 'Citations', href: '#' },
        { name: 'Embeddings', href: '#' },
      ],
    },
    {
      title: 'API Reference',
      icon: Code,
      links: [
        { name: 'REST API', href: '#' },
        { name: 'gRPC API', href: '#' },
        { name: 'Client SDKs', href: '#' },
        { name: 'Authentication', href: '#' },
      ],
    },
    {
      title: 'Deployment',
      icon: Cloud,
      links: [
        { name: 'On-Premise Setup', href: '#' },
        { name: 'Docker Deployment', href: '#' },
        { name: 'Kubernetes', href: '#' },
        { name: 'Load Balancing', href: '#' },
      ],
    },
    {
      title: 'Configuration',
      icon: Settings,
      links: [
        { name: 'Storage Backends', href: '#' },
        { name: 'Performance Tuning', href: '#' },
        { name: 'Security Settings', href: '#' },
        { name: 'Monitoring', href: '#' },
      ],
    },
    {
      title: 'Advanced',
      icon: Zap,
      links: [
        { name: 'Custom Embeddings', href: '#' },
        { name: 'Hybrid Search', href: '#' },
        { name: 'Multi-Tenancy', href: '#' },
        { name: 'Scaling', href: '#' },
      ],
    },
  ]

  const quickLinks = [
    {
      icon: Terminal,
      title: 'CLI Reference',
      description: 'Complete command-line interface documentation',
      link: '#',
    },
    {
      icon: FileText,
      title: 'Changelog',
      description: 'Release notes and version history',
      link: '#',
    },
    {
      icon: Lock,
      title: 'Security',
      description: 'Security best practices and compliance',
      link: '#',
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
            Documentation
          </h1>
          <p className="text-xl text-slate-600 dark:text-slate-400 max-w-3xl mx-auto mb-8">
            Everything you need to deploy and run SHODH in production
          </p>
          <div className="flex flex-col sm:flex-row gap-4 justify-center">
            <a
              href="#"
              className="px-8 py-4 bg-primary hover:bg-primary/90 text-white rounded-lg font-semibold transition-all transform hover:scale-105 flex items-center justify-center gap-2"
            >
              <Book className="w-5 h-5" />
              Read Full Docs
            </a>
            <Link
              href="/demo"
              className="px-8 py-4 bg-white dark:bg-slate-900 border border-slate-300 dark:border-slate-700 rounded-lg font-semibold hover:border-primary dark:hover:border-primary transition-colors text-slate-900 dark:text-white"
            >
              Try Demo
            </Link>
          </div>
        </motion.div>

        {/* Main Doc Sections */}
        <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-6 max-w-6xl mx-auto mb-16">
          {docSections.map((section, index) => (
            <motion.div
              key={section.title}
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.5, delay: index * 0.1 }}
              className="p-6 rounded-xl border border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900 hover:border-primary dark:hover:border-primary transition-all"
            >
              <div className="flex items-center gap-3 mb-4">
                <div className="w-10 h-10 bg-primary/10 dark:bg-primary/20 rounded-lg flex items-center justify-center">
                  <section.icon className="w-5 h-5 text-primary" />
                </div>
                <h3 className="text-xl font-semibold text-slate-900 dark:text-white">
                  {section.title}
                </h3>
              </div>
              <ul className="space-y-2">
                {section.links.map((link) => (
                  <li key={link.name}>
                    <a
                      href={link.href}
                      className="text-slate-600 dark:text-slate-400 hover:text-primary dark:hover:text-primary transition-colors flex items-center gap-2 group"
                    >
                      <ArrowRight className="w-4 h-4 opacity-0 group-hover:opacity-100 transition-opacity" />
                      {link.name}
                    </a>
                  </li>
                ))}
              </ul>
            </motion.div>
          ))}
        </div>

        {/* Quick Links */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          viewport={{ once: true }}
          className="max-w-6xl mx-auto mb-16"
        >
          <h2 className="text-3xl font-bold mb-8 text-center text-slate-900 dark:text-white">
            Quick Links
          </h2>
          <div className="grid md:grid-cols-3 gap-6">
            {quickLinks.map((link, index) => (
              <motion.a
                key={link.title}
                href={link.link}
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                transition={{ delay: index * 0.1 }}
                viewport={{ once: true }}
                className="p-6 rounded-xl border border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900 hover:border-primary dark:hover:border-primary hover:shadow-lg transition-all"
              >
                <div className="w-12 h-12 bg-primary/10 dark:bg-primary/20 rounded-lg flex items-center justify-center mb-4">
                  <link.icon className="w-6 h-6 text-primary" />
                </div>
                <h3 className="text-lg font-semibold mb-2 text-slate-900 dark:text-white">
                  {link.title}
                </h3>
                <p className="text-sm text-slate-600 dark:text-slate-400">
                  {link.description}
                </p>
              </motion.a>
            ))}
          </div>
        </motion.div>

        {/* Beta Notice */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          viewport={{ once: true }}
          className="max-w-4xl mx-auto"
        >
          <div className="rounded-xl border-2 border-primary/30 bg-gradient-to-br from-primary/5 to-primary/10 dark:from-primary/10 dark:to-primary/20 p-8 text-center">
            <div className="inline-flex items-center gap-2 px-4 py-2 bg-primary/20 dark:bg-primary/30 rounded-full text-primary font-semibold mb-4">
              <Rocket className="w-5 h-5" />
              Beta Coming Soon
            </div>
            <h2 className="text-2xl font-bold mb-4 text-slate-900 dark:text-white">
              Documentation Under Development
            </h2>
            <p className="text-lg text-slate-600 dark:text-slate-400 mb-6">
              Full documentation will be available when we launch the beta.
              Contact us to get early access and work directly with our team.
            </p>
            <div className="flex flex-col sm:flex-row gap-4 justify-center">
              <Link
                href="/getting-started"
                className="px-8 py-4 bg-primary hover:bg-primary/90 text-white rounded-lg font-semibold transition-all transform hover:scale-105"
              >
                Request Beta Access
              </Link>
              <a
                href="mailto:29.varun@gmail.com"
                className="px-8 py-4 bg-white dark:bg-slate-900 border border-slate-300 dark:border-slate-700 rounded-lg font-semibold hover:border-primary dark:hover:border-primary transition-colors text-slate-900 dark:text-white"
              >
                Contact Us
              </a>
            </div>
          </div>
        </motion.div>

        {/* CTA */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          viewport={{ once: true }}
          className="text-center mt-16 p-12 rounded-2xl bg-gradient-to-br from-primary/10 via-secondary/10 to-destructive/10 dark:from-primary/20 dark:via-secondary/20 dark:to-destructive/20"
        >
          <h2 className="text-3xl font-bold mb-4 text-slate-900 dark:text-white">
            Need Help?
          </h2>
          <p className="text-xl text-slate-600 dark:text-slate-400 mb-8">
            Join our community or contact support for personalized assistance
          </p>
          <div className="flex flex-col sm:flex-row gap-4 justify-center">
            <a
              href="#"
              className="px-8 py-4 bg-primary hover:bg-primary/90 text-white rounded-lg font-semibold transition-all transform hover:scale-105"
            >
              Join Discord
            </a>
            <a
              href="#"
              className="px-8 py-4 bg-white dark:bg-slate-900 border border-slate-300 dark:border-slate-700 rounded-lg font-semibold hover:border-primary dark:hover:border-primary transition-colors text-slate-900 dark:text-white"
            >
              Contact Support
            </a>
          </div>
        </motion.div>
      </div>
    </main>
  )
}
