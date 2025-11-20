'use client'

import { motion } from 'framer-motion'
import {
  Rocket, Zap, Database, Brain, Mail, Phone
} from 'lucide-react'
import Link from 'next/link'

export default function GettingStarted() {
  const systemRequirements = [
    { icon: Zap, label: 'CPU', value: 'i5/i7 or equivalent (no GPU needed)' },
    { icon: Database, label: 'RAM', value: '16GB (8GB minimum)' },
    { icon: Brain, label: 'Storage', value: '2GB free space' },
  ]

  const features = [
    '✅ Index 500-1000 documents on laptop',
    '✅ 50-80ms query latency',
    '✅ Hybrid search (Vamana + BM25)',
    '✅ LLM flexibility (local or API)',
    '✅ Multi-language support (Hindi, Tamil, etc.)',
    '✅ Offline-first operation',
  ]

  return (
    <main className="min-h-screen bg-gradient-to-b from-white via-slate-50 to-white dark:from-slate-950 dark:via-slate-900 dark:to-slate-950">
      <div className="container mx-auto px-4 py-24">
        {/* Hero */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          className="text-center mb-16"
        >
          <div className="inline-flex items-center gap-2 px-4 py-2 bg-primary/10 dark:bg-primary/20 rounded-full text-primary font-semibold mb-6">
            <Rocket className="w-5 h-5" />
            Beta Coming Soon
          </div>
          <h1 className="text-5xl md:text-6xl font-bold mb-6 text-slate-900 dark:text-white">
            Join the Shodh RAG Beta
          </h1>
          <p className="text-xl text-slate-600 dark:text-slate-400 max-w-3xl mx-auto mb-8">
            Be among the first to run production RAG on your laptop.
            We're currently in private beta and will be launching soon.
          </p>
        </motion.div>

        {/* System Requirements */}
        <section className="mb-16">
          <h2 className="text-3xl font-bold mb-8 text-center text-slate-900 dark:text-white">
            What You'll Need
          </h2>
          <div className="grid md:grid-cols-3 gap-6 max-w-4xl mx-auto">
            {systemRequirements.map((req, index) => (
              <motion.div
                key={req.label}
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                transition={{ delay: index * 0.1 }}
                viewport={{ once: true }}
                className="p-6 rounded-xl border border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900 text-center"
              >
                <div className="w-12 h-12 bg-primary/10 dark:bg-primary/20 rounded-lg flex items-center justify-center mx-auto mb-4">
                  <req.icon className="w-6 h-6 text-primary" />
                </div>
                <h3 className="font-semibold mb-2 text-slate-900 dark:text-white">{req.label}</h3>
                <p className="text-sm text-slate-600 dark:text-slate-400">{req.value}</p>
              </motion.div>
            ))}
          </div>
        </section>

        {/* What You Get */}
        <section className="mb-16">
          <h2 className="text-3xl font-bold mb-8 text-center text-slate-900 dark:text-white">
            What You'll Get in Beta
          </h2>
          <div className="grid md:grid-cols-2 gap-4 max-w-3xl mx-auto">
            {features.map((feature, index) => (
              <motion.div
                key={index}
                initial={{ opacity: 0, scale: 0.95 }}
                whileInView={{ opacity: 1, scale: 1 }}
                transition={{ delay: index * 0.05 }}
                viewport={{ once: true }}
                className="p-4 rounded-lg border border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900 text-slate-700 dark:text-slate-300"
              >
                {feature}
              </motion.div>
            ))}
          </div>
        </section>

        {/* Contact Section */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          viewport={{ once: true }}
          className="max-w-3xl mx-auto p-8 rounded-2xl bg-gradient-to-br from-primary/10 via-secondary/10 to-destructive/10 dark:from-primary/20 dark:via-secondary/20 dark:to-destructive/20 text-center"
        >
          <h2 className="text-3xl font-bold mb-4 text-slate-900 dark:text-white">
            Request Beta Access
          </h2>
          <p className="text-lg text-slate-600 dark:text-slate-400 mb-8 max-w-2xl mx-auto">
            Contact us to learn more about Shodh RAG and get early access to the beta program.
          </p>

          <div className="flex flex-col sm:flex-row gap-6 justify-center mb-8">
            <a
              href="mailto:29.varun@gmail.com"
              className="inline-flex items-center gap-3 px-6 py-3 bg-white dark:bg-slate-900 border-2 border-slate-200 dark:border-slate-800 rounded-lg hover:border-primary dark:hover:border-primary transition-colors text-slate-900 dark:text-white"
            >
              <Mail className="w-5 h-5 text-primary" />
              <div className="text-left">
                <div className="text-xs text-slate-500 dark:text-slate-400">Email</div>
                <div className="font-semibold">29.varun@gmail.com</div>
              </div>
            </a>

            <a
              href="tel:+919810300618"
              className="inline-flex items-center gap-3 px-6 py-3 bg-white dark:bg-slate-900 border-2 border-slate-200 dark:border-slate-800 rounded-lg hover:border-primary dark:hover:border-primary transition-colors text-slate-900 dark:text-white"
            >
              <Phone className="w-5 h-5 text-primary" />
              <div className="text-left">
                <div className="text-xs text-slate-500 dark:text-slate-400">Phone</div>
                <div className="font-semibold">+91 98103 00618</div>
              </div>
            </a>
          </div>

          <div className="flex flex-col sm:flex-row gap-4 justify-center">
            <Link
              href="/pricing"
              className="px-8 py-4 bg-primary hover:bg-primary/90 text-white rounded-lg font-semibold transition-all transform hover:scale-105"
            >
              View Pricing
            </Link>
            <Link
              href="/demo"
              className="px-8 py-4 border-2 border-slate-300 dark:border-slate-700 rounded-lg font-semibold hover:border-primary dark:hover:border-primary transition-colors text-slate-900 dark:text-white"
            >
              Schedule a Demo
            </Link>
          </div>
        </motion.div>

        {/* Why Join Beta */}
        <section className="mt-16 max-w-4xl mx-auto">
          <h2 className="text-3xl font-bold mb-8 text-center text-slate-900 dark:text-white">
            Why Join the Beta?
          </h2>
          <div className="grid md:grid-cols-3 gap-6">
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              viewport={{ once: true }}
              className="p-6 rounded-xl border border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900"
            >
              <div className="text-4xl mb-4">🚀</div>
              <h3 className="text-xl font-semibold mb-2 text-slate-900 dark:text-white">Early Access</h3>
              <p className="text-slate-600 dark:text-slate-400">
                Be first to experience production RAG on your laptop before public launch
              </p>
            </motion.div>

            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              transition={{ delay: 0.1 }}
              viewport={{ once: true }}
              className="p-6 rounded-xl border border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900"
            >
              <div className="text-4xl mb-4">💬</div>
              <h3 className="text-xl font-semibold mb-2 text-slate-900 dark:text-white">Direct Support</h3>
              <p className="text-slate-600 dark:text-slate-400">
                Work directly with our team to solve your RAG use cases
              </p>
            </motion.div>

            <motion.div
              initial={{ opacity: 0, y: 20 }}
              whileInView={{ opacity: 1, y: 0 }}
              transition={{ delay: 0.2 }}
              viewport={{ once: true }}
              className="p-6 rounded-xl border border-slate-200 dark:border-slate-800 bg-white dark:bg-slate-900"
            >
              <div className="text-4xl mb-4">🎁</div>
              <h3 className="text-xl font-semibold mb-2 text-slate-900 dark:text-white">Special Pricing</h3>
              <p className="text-slate-600 dark:text-slate-400">
                Beta users get preferential pricing when we launch publicly
              </p>
            </motion.div>
          </div>
        </section>
      </div>
    </main>
  )
}
