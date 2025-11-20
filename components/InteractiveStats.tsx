'use client'

import { motion, useInView, useMotionValue, useSpring } from 'framer-motion'
import { useEffect, useRef } from 'react'

interface Stat {
  value: string
  label: string
  suffix?: string
}

export default function InteractiveStats({ stats }: { stats: Stat[] }) {
  return (
    <div className="grid grid-cols-2 md:grid-cols-4 gap-8 max-w-4xl mx-auto">
      {stats.map((stat, index) => (
        <AnimatedStat key={stat.label} stat={stat} index={index} />
      ))}
    </div>
  )
}

function AnimatedStat({ stat, index }: { stat: Stat; index: number }) {
  const ref = useRef(null)
  const isInView = useInView(ref, { once: true })

  return (
    <motion.div
      ref={ref}
      initial={{ opacity: 0, y: 20 }}
      animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 20 }}
      transition={{ duration: 0.5, delay: 0.3 + index * 0.1 }}
      className="text-center group cursor-pointer"
    >
      <motion.div
        className="text-3xl md:text-4xl font-bold text-primary mb-2 group-hover:scale-110 transition-transform"
        whileHover={{ scale: 1.1 }}
      >
        {stat.value}
      </motion.div>
      <div className="text-sm text-slate-600 dark:text-slate-400">{stat.label}</div>
    </motion.div>
  )
}
