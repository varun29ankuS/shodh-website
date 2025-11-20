'use client'

import { motion } from 'framer-motion'
import { Check, X } from 'lucide-react'
import { useState } from 'react'

interface ComparisonFeature {
  name: string
  shodh: boolean | string
  cloudApis: boolean | string
}

const features: ComparisonFeature[] = [
  { name: 'Data Privacy', shodh: 'On your servers', cloudApis: 'Sent to cloud' },
  { name: 'Pricing Model', shodh: 'One-time license', cloudApis: 'Pay per token' },
  { name: 'Citation Accuracy', shodh: '97%', cloudApis: 'Limited' },
  { name: 'Vendor Lock-in', shodh: false, cloudApis: true },
  { name: 'Source Code Access', shodh: true, cloudApis: false },
  { name: 'Unlimited Queries', shodh: true, cloudApis: false },
  { name: 'Custom Development', shodh: true, cloudApis: false },
  { name: 'SLA Guarantees', shodh: true, cloudApis: 'Extra cost' },
]

export default function InteractiveComparison() {
  const [hoveredRow, setHoveredRow] = useState<number | null>(null)

  return (
    <div className="overflow-x-auto">
      <table className="w-full border-collapse">
        <thead>
          <tr className="border-b-2 border-slate-200 dark:border-slate-800">
            <th className="text-left p-4 text-slate-900 dark:text-white font-semibold">Feature</th>
            <th className="text-center p-4 text-primary font-semibold">SHODH</th>
            <th className="text-center p-4 text-slate-600 dark:text-slate-400 font-semibold">Cloud APIs</th>
          </tr>
        </thead>
        <tbody>
          {features.map((feature, index) => (
            <motion.tr
              key={feature.name}
              initial={{ opacity: 0, x: -20 }}
              whileInView={{ opacity: 1, x: 0 }}
              transition={{ delay: index * 0.05 }}
              viewport={{ once: true }}
              onHoverStart={() => setHoveredRow(index)}
              onHoverEnd={() => setHoveredRow(null)}
              className={`border-b border-slate-200 dark:border-slate-800 transition-colors ${
                hoveredRow === index ? 'bg-primary/5' : ''
              }`}
            >
              <td className="p-4 text-slate-900 dark:text-white font-medium">{feature.name}</td>
              <td className="p-4 text-center">
                {typeof feature.shodh === 'boolean' ? (
                  feature.shodh ? (
                    <Check className="w-5 h-5 text-green-500 mx-auto" />
                  ) : (
                    <X className="w-5 h-5 text-red-500 mx-auto" />
                  )
                ) : (
                  <span className="text-primary font-semibold">{feature.shodh}</span>
                )}
              </td>
              <td className="p-4 text-center opacity-60">
                {typeof feature.cloudApis === 'boolean' ? (
                  feature.cloudApis ? (
                    <Check className="w-5 h-5 text-green-500 mx-auto" />
                  ) : (
                    <X className="w-5 h-5 text-red-500 mx-auto" />
                  )
                ) : (
                  <span className="text-slate-600 dark:text-slate-400">{feature.cloudApis}</span>
                )}
              </td>
            </motion.tr>
          ))}
        </tbody>
      </table>
    </div>
  )
}
