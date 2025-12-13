'use client'

import { motion } from 'framer-motion'
import { Check, ArrowRight, Brain, FileSearch, MessageCircle, Github, ExternalLink } from 'lucide-react'
import Link from 'next/link'

export default function Pricing() {
  const memoryTiers = [
    {
      name: 'Community',
      price: 'Free',
      period: 'forever',
      description: 'Open source. Self-hosted. Full features.',
      features: [
        'All core features',
        'Unlimited memories',
        'Semantic + associative recall',
        'MCP server for Claude',
        'Python SDK',
        'Community support (GitHub)',
      ],
      cta: 'Get Started',
      ctaLink: '/docs',
      highlighted: false,
      badge: 'Open Source',
    },
    {
      name: 'Enterprise',
      price: 'Contact Us',
      period: 'custom pricing',
      description: 'For teams needing support and SLAs.',
      features: [
        'Everything in Community',
        'Priority support',
        'SLA guarantees',
        'Custom integrations',
        'Training & onboarding',
        'Dedicated Slack/Teams channel',
      ],
      cta: 'Contact Sales',
      ctaLink: 'https://wa.me/919810300618?text=Hi, I\'m interested in Shodh Memory Enterprise',
      highlighted: true,
      badge: null,
    },
  ]

  const ragTiers = [
    {
      name: 'Pilot',
      price: 'Free',
      period: 'to evaluate',
      description: 'Try Shodh RAG on your documents.',
      features: [
        'Up to 100 documents',
        'Full RAG pipeline',
        'Citation with sources',
        'Local or cloud LLM',
        'Community support',
        '30-day pilot period',
      ],
      cta: 'Start Pilot',
      ctaLink: 'https://wa.me/919810300618?text=Hi, I\'d like to start a Shodh RAG pilot',
      highlighted: false,
      badge: 'Try Free',
    },
    {
      name: 'Enterprise',
      price: 'Contact Us',
      period: 'custom pricing',
      description: 'Production deployment with full support.',
      features: [
        'Unlimited documents',
        'On-premise deployment',
        'Priority support + SLA',
        'Custom document parsers',
        'Advanced security features',
        'Training & onboarding',
      ],
      cta: 'Contact Sales',
      ctaLink: 'https://wa.me/919810300618?text=Hi, I\'m interested in Shodh RAG Enterprise',
      highlighted: true,
      badge: null,
    },
  ]

  const faqs = [
    {
      q: 'Is Shodh Memory really free?',
      a: 'Yes. Shodh Memory is open source (MIT license). Install from PyPI or npm and use it forever. Enterprise tier is for organizations that need priority support, SLAs, and custom integrations.',
    },
    {
      q: 'What\'s included in the Shodh RAG pilot?',
      a: 'We\'ll help you set up Shodh RAG on your infrastructure with up to 100 of your documents. You get 30 days to evaluate with full features. No payment required to start.',
    },
    {
      q: 'Can I run everything on-premise?',
      a: 'Yes. Both products are designed for on-premise deployment. Your data never leaves your infrastructure. No cloud dependency required.',
    },
    {
      q: 'What are the hardware requirements?',
      a: 'Shodh Memory runs on any machine. Shodh RAG needs 16GB+ RAM for production workloads. No GPU required. Works on standard laptops for evaluation.',
    },
    {
      q: 'Do you offer custom development?',
      a: 'Yes. Enterprise plans include custom integrations, document parsers, and feature development based on your requirements.',
    },
  ]

  return (
    <main className="min-h-screen bg-gradient-to-b from-white via-slate-50 to-white dark:from-slate-950 dark:via-slate-900 dark:to-slate-950">
      <div className="container mx-auto px-4 py-24">
        {/* Header */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          className="text-center mb-20"
        >
          <h1 className="text-5xl md:text-6xl font-bold mb-6 text-slate-900 dark:text-white">
            Pricing
          </h1>
          <p className="text-xl text-slate-600 dark:text-slate-400 max-w-2xl mx-auto">
            Two products. Simple pricing. Start free, scale when ready.
          </p>
        </motion.div>

        {/* Shodh Memory Section */}
        <motion.section
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          viewport={{ once: true }}
          className="mb-24"
        >
          <div className="text-center mb-12">
            <div className="inline-flex items-center gap-2 px-4 py-2 bg-primary/10 dark:bg-primary/20 rounded-full mb-4">
              <Brain className="w-5 h-5 text-primary" />
              <span className="font-semibold text-primary">Shodh Memory</span>
            </div>
            <h2 className="text-3xl font-bold mb-3 text-slate-900 dark:text-white">
              Persistent Memory for AI Agents
            </h2>
            <p className="text-slate-600 dark:text-slate-400 max-w-xl mx-auto">
              Give your AI agents memory that learns and persists across sessions.
              Works with Claude Code, Claude Desktop, and any MCP-compatible client.
            </p>
          </div>

          <div className="grid md:grid-cols-2 gap-8 max-w-4xl mx-auto">
            {memoryTiers.map((tier, index) => (
              <motion.div
                key={tier.name}
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                transition={{ delay: index * 0.1 }}
                viewport={{ once: true }}
                className={`p-8 rounded-2xl border-2 relative ${
                  tier.highlighted
                    ? 'border-primary bg-gradient-to-br from-primary/5 to-primary/10 dark:from-primary/10 dark:to-primary/20'
                    : 'border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900'
                }`}
              >
                {tier.badge && (
                  <div className="absolute -top-3 left-6 px-3 py-1 bg-green-500 text-white text-xs font-bold rounded-full">
                    {tier.badge}
                  </div>
                )}

                <h3 className="text-2xl font-bold mb-2 text-slate-900 dark:text-white">
                  {tier.name}
                </h3>
                <div className="mb-4">
                  <span className="text-4xl font-bold text-primary">{tier.price}</span>
                  <span className="text-slate-500 dark:text-slate-400 ml-2">{tier.period}</span>
                </div>
                <p className="text-slate-600 dark:text-slate-400 mb-6">
                  {tier.description}
                </p>

                <a
                  href={tier.ctaLink}
                  target={tier.ctaLink.startsWith('http') ? '_blank' : undefined}
                  rel={tier.ctaLink.startsWith('http') ? 'noopener noreferrer' : undefined}
                  className={`block w-full py-3 px-6 rounded-lg font-semibold text-center mb-8 transition-all transform hover:scale-105 ${
                    tier.highlighted
                      ? 'bg-primary hover:bg-primary/90 text-white'
                      : 'bg-slate-100 dark:bg-slate-800 text-slate-900 dark:text-white hover:bg-primary hover:text-white'
                  }`}
                >
                  {tier.cta}
                </a>

                <ul className="space-y-3">
                  {tier.features.map((feature) => (
                    <li key={feature} className="flex items-start gap-3">
                      <Check className="w-5 h-5 text-primary flex-shrink-0 mt-0.5" />
                      <span className="text-slate-700 dark:text-slate-300">{feature}</span>
                    </li>
                  ))}
                </ul>

                {tier.name === 'Community' && (
                  <div className="mt-6 pt-6 border-t border-slate-200 dark:border-slate-700 flex gap-3">
                    <a
                      href="https://pypi.org/project/shodh-memory/"
                      target="_blank"
                      rel="noopener noreferrer"
                      className="flex items-center gap-1 text-sm text-slate-500 hover:text-primary transition-colors"
                    >
                      PyPI <ExternalLink className="w-3 h-3" />
                    </a>
                    <a
                      href="https://www.npmjs.com/package/@shodh/memory-mcp"
                      target="_blank"
                      rel="noopener noreferrer"
                      className="flex items-center gap-1 text-sm text-slate-500 hover:text-primary transition-colors"
                    >
                      npm <ExternalLink className="w-3 h-3" />
                    </a>
                    <a
                      href="https://github.com/varun29ankuS/shodh-memory"
                      target="_blank"
                      rel="noopener noreferrer"
                      className="flex items-center gap-1 text-sm text-slate-500 hover:text-primary transition-colors"
                    >
                      <Github className="w-4 h-4" /> GitHub
                    </a>
                  </div>
                )}
              </motion.div>
            ))}
          </div>
        </motion.section>

        {/* Shodh RAG Section */}
        <motion.section
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          viewport={{ once: true }}
          className="mb-24"
        >
          <div className="text-center mb-12">
            <div className="inline-flex items-center gap-2 px-4 py-2 bg-secondary/10 dark:bg-secondary/20 rounded-full mb-4">
              <FileSearch className="w-5 h-5 text-secondary" />
              <span className="font-semibold text-secondary">Shodh RAG</span>
            </div>
            <h2 className="text-3xl font-bold mb-3 text-slate-900 dark:text-white">
              Search Your Documents with AI
            </h2>
            <p className="text-slate-600 dark:text-slate-400 max-w-xl mx-auto">
              Ask questions, get answers with citations. 97% accuracy.
              On-premise deployment. Your data stays with you.
            </p>
          </div>

          <div className="grid md:grid-cols-2 gap-8 max-w-4xl mx-auto">
            {ragTiers.map((tier, index) => (
              <motion.div
                key={tier.name}
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                transition={{ delay: index * 0.1 }}
                viewport={{ once: true }}
                className={`p-8 rounded-2xl border-2 relative ${
                  tier.highlighted
                    ? 'border-secondary bg-gradient-to-br from-secondary/5 to-secondary/10 dark:from-secondary/10 dark:to-secondary/20'
                    : 'border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900'
                }`}
              >
                {tier.badge && (
                  <div className="absolute -top-3 left-6 px-3 py-1 bg-green-500 text-white text-xs font-bold rounded-full">
                    {tier.badge}
                  </div>
                )}

                <h3 className="text-2xl font-bold mb-2 text-slate-900 dark:text-white">
                  {tier.name}
                </h3>
                <div className="mb-4">
                  <span className="text-4xl font-bold text-secondary">{tier.price}</span>
                  <span className="text-slate-500 dark:text-slate-400 ml-2">{tier.period}</span>
                </div>
                <p className="text-slate-600 dark:text-slate-400 mb-6">
                  {tier.description}
                </p>

                <a
                  href={tier.ctaLink}
                  target="_blank"
                  rel="noopener noreferrer"
                  className={`block w-full py-3 px-6 rounded-lg font-semibold text-center mb-8 transition-all transform hover:scale-105 ${
                    tier.highlighted
                      ? 'bg-secondary hover:bg-secondary/90 text-white'
                      : 'bg-slate-100 dark:bg-slate-800 text-slate-900 dark:text-white hover:bg-secondary hover:text-white'
                  }`}
                >
                  {tier.cta}
                </a>

                <ul className="space-y-3">
                  {tier.features.map((feature) => (
                    <li key={feature} className="flex items-start gap-3">
                      <Check className="w-5 h-5 text-secondary flex-shrink-0 mt-0.5" />
                      <span className="text-slate-700 dark:text-slate-300">{feature}</span>
                    </li>
                  ))}
                </ul>
              </motion.div>
            ))}
          </div>
        </motion.section>

        {/* FAQ */}
        <motion.section
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          viewport={{ once: true }}
          className="max-w-3xl mx-auto mb-16"
        >
          <h2 className="text-3xl font-bold mb-8 text-center text-slate-900 dark:text-white">
            Frequently Asked Questions
          </h2>
          <div className="space-y-4">
            {faqs.map((faq, index) => (
              <motion.div
                key={index}
                initial={{ opacity: 0, y: 10 }}
                whileInView={{ opacity: 1, y: 0 }}
                transition={{ delay: index * 0.05 }}
                viewport={{ once: true }}
                className="p-6 rounded-xl border border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900"
              >
                <h3 className="text-lg font-semibold mb-2 text-slate-900 dark:text-white">
                  {faq.q}
                </h3>
                <p className="text-slate-600 dark:text-slate-400">{faq.a}</p>
              </motion.div>
            ))}
          </div>
        </motion.section>

        {/* CTA */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          viewport={{ once: true }}
          className="text-center p-12 rounded-2xl bg-gradient-to-br from-primary/10 via-secondary/10 to-destructive/10 dark:from-primary/20 dark:via-secondary/20 dark:to-destructive/20"
        >
          <h2 className="text-3xl font-bold mb-4 text-slate-900 dark:text-white">
            Not sure which product you need?
          </h2>
          <p className="text-xl text-slate-600 dark:text-slate-400 mb-8 max-w-xl mx-auto">
            Let's talk. We'll help you figure out the right solution.
          </p>
          <div className="flex flex-col sm:flex-row gap-4 justify-center">
            <a
              href="https://wa.me/919810300618?text=Hi, I need help choosing the right Shodh product"
              target="_blank"
              rel="noopener noreferrer"
              className="px-8 py-4 bg-primary hover:bg-primary/90 text-white rounded-lg font-semibold transition-all transform hover:scale-105 flex items-center justify-center gap-2"
            >
              <MessageCircle className="w-5 h-5" />
              WhatsApp Us
            </a>
            <Link
              href="/demo"
              className="px-8 py-4 bg-white dark:bg-slate-900 border border-slate-300 dark:border-slate-700 rounded-lg font-semibold hover:border-primary dark:hover:border-primary transition-colors text-slate-900 dark:text-white flex items-center justify-center gap-2"
            >
              Book a Demo
              <ArrowRight className="w-5 h-5" />
            </Link>
          </div>
        </motion.div>
      </div>
    </main>
  )
}
