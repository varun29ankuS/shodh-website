'use client'

import { motion } from 'framer-motion'
import { Check, ArrowRight, Zap } from 'lucide-react'
import Link from 'next/link'

export default function Pricing() {
  const tiers = [
    {
      name: 'Starter',
      price: 'Contact Us',
      period: 'for pricing',
      description: 'Perfect for individuals and small teams getting started',
      features: [
        'Up to 500 documents',
        'Core RAG features',
        'Hybrid search (Vamana + BM25)',
        'LLM flexibility (local/API)',
        'Community support',
        'Email support',
      ],
      cta: 'Contact Us',
      highlighted: false,
    },
    {
      name: 'Professional',
      price: 'Contact Us',
      period: 'for pricing',
      description: 'For teams with growing document collections',
      features: [
        'Up to 2,000 documents',
        'All Starter features',
        'Priority email support',
        'Advanced filtering',
        'Query optimization',
        'Performance monitoring',
        'Slack/Teams integration',
      ],
      cta: 'Contact Us',
      highlighted: true,
    },
    {
      name: 'Enterprise',
      price: 'Contact Us',
      period: 'custom solution',
      description: 'For organizations with mission-critical requirements',
      features: [
        'Unlimited documents',
        'All Professional features',
        'On-premise deployment support',
        'SLA guarantees',
        'Dedicated support',
        'Training & onboarding',
        'Custom feature development',
      ],
      cta: 'Contact Sales',
      highlighted: false,
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
            Simple, Transparent Pricing
          </h1>
          <p className="text-xl text-slate-600 dark:text-slate-400 max-w-2xl mx-auto">
            Start free. Scale as you grow. No cloud infrastructure costs.
          </p>
        </motion.div>

        <div className="grid md:grid-cols-3 gap-8 max-w-7xl mx-auto mb-16">
          {tiers.map((tier, index) => (
            <motion.div
              key={tier.name}
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.5, delay: index * 0.1 }}
              className={`p-8 rounded-2xl border-2 ${
                tier.highlighted
                  ? 'border-primary bg-gradient-to-br from-primary/5 to-primary/10 dark:from-primary/10 dark:to-primary/20 shadow-xl scale-105'
                  : 'border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900'
              }`}
            >
              {tier.highlighted && (
                <div className="flex items-center gap-2 px-3 py-1 bg-primary text-white rounded-full text-sm font-semibold mb-4 w-fit">
                  <Zap className="w-4 h-4" />
                  Most Popular
                </div>
              )}

              <h3 className="text-2xl font-bold mb-2 text-slate-900 dark:text-white">{tier.name}</h3>
              <div className="mb-4">
                <span className="text-3xl font-bold text-primary">{tier.price}</span>
                <div className="text-sm text-slate-600 dark:text-slate-400 mt-1">{tier.period}</div>
              </div>
              <p className="text-slate-600 dark:text-slate-400 mb-6">{tier.description}</p>

              <Link
                href="/demo"
                className={`block w-full py-3 px-6 rounded-lg font-semibold text-center mb-8 transition-all transform hover:scale-105 ${
                  tier.highlighted
                    ? 'bg-primary hover:bg-primary/90 text-white'
                    : 'bg-slate-100 dark:bg-slate-800 text-slate-900 dark:text-white hover:bg-primary hover:text-white dark:hover:bg-primary'
                }`}
              >
                {tier.cta}
              </Link>

              <ul className="space-y-4">
                {tier.features.map((feature) => (
                  <li key={feature} className="flex items-start gap-3">
                    <Check className="w-5 h-5 text-primary flex-shrink-0 mt-0.5" />
                    <span className="text-slate-700 dark:text-slate-300">{feature}</span>
                  </li>
                ))}
              </ul>
            </motion.div>
          ))}
        </div>

        {/* FAQ */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          viewport={{ once: true }}
          className="max-w-3xl mx-auto"
        >
          <h2 className="text-3xl font-bold mb-8 text-center text-slate-900 dark:text-white">
            Frequently Asked Questions
          </h2>
          <div className="space-y-6">
            {[
              {
                q: 'How many documents can I index on a laptop?',
                a: '500-1000 documents comfortably on a 16GB laptop. This uses ~190MB RAM and indexes overnight (60-90 minutes). Larger collections need desktop/server hardware.',
              },
              {
                q: 'Can I use local LLMs or must I pay for APIs?',
                a: 'Your choice! Run local models (llama.cpp/ONNX) for free, or use cloud APIs (GPT-4, Claude, Gemini) for better quality. Mix and match based on your budget.',
              },
              {
                q: 'What are the infrastructure requirements?',
                a: 'Runs on existing laptops/PCs (16GB RAM, i5/i7 CPU). No GPU required. No cloud infrastructure needed. Your data stays local.',
              },
              {
                q: 'Is support included?',
                a: 'Free tier has community support. Professional and Enterprise tiers include email and priority support with SLA guarantees.',
              },
            ].map((faq, index) => (
              <motion.div
                key={index}
                initial={{ opacity: 0, x: -20 }}
                whileInView={{ opacity: 1, x: 0 }}
                transition={{ delay: index * 0.1 }}
                viewport={{ once: true }}
                className="p-6 rounded-xl border border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900"
              >
                <h3 className="text-lg font-semibold mb-2 text-slate-900 dark:text-white">{faq.q}</h3>
                <p className="text-slate-600 dark:text-slate-400">{faq.a}</p>
              </motion.div>
            ))}
          </div>
        </motion.div>

        {/* CTA */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          viewport={{ once: true }}
          className="text-center mt-16"
        >
          <h2 className="text-3xl font-bold mb-4 text-slate-900 dark:text-white">
            Still have questions?
          </h2>
          <p className="text-xl text-slate-600 dark:text-slate-400 mb-8">
            Talk to our team to find the right plan for you
          </p>
          <Link
            href="/demo"
            className="inline-flex items-center gap-2 px-8 py-4 bg-primary hover:bg-primary/90 text-white rounded-lg font-semibold transition-all transform hover:scale-105"
          >
            Schedule a Demo
            <ArrowRight className="w-5 h-5" />
          </Link>
        </motion.div>
      </div>
    </main>
  )
}
