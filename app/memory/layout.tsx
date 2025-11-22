import { Metadata } from 'next'

export const metadata: Metadata = {
  title: 'AI Memory for Edge Devices - Robots, Drones & IoT | Shodh',
  description: 'Edge-native AI memory system for robots, drones, and IoT devices. 100% offline, 4MB binary, <100ms retrieval. 3-tier memory hierarchy with knowledge graphs. Built for defence, industrial, and autonomous systems.',
  keywords: [
    'edge AI memory', 'robot memory system', 'drone AI', 'IoT memory',
    'offline AI', 'air-gapped AI', 'defence AI', 'military AI',
    'industrial robotics', 'autonomous drones', 'embedded AI',
    'local knowledge graph', 'edge computing', 'RocksDB', 'Rust AI',
    'Graphiti knowledge graph', 'multi-tier memory', 'persistent memory',
    'robot knowledge base', 'IoT intelligence', 'edge memory system'
  ],
  openGraph: {
    title: 'AI Memory for Edge Devices - Robots, Drones & IoT',
    description: '100% offline AI memory for robots and drones. 4MB binary, <100ms retrieval, works without internet. Perfect for defence, agriculture, and industrial automation.',
    type: 'website',
    url: 'https://shodh-rag.com/memory',
    images: [
      {
        url: '/og-image.png',
        width: 1200,
        height: 630,
        alt: 'Shodh Memory - Edge AI Memory for Robots, Drones & IoT',
      },
    ],
  },
  twitter: {
    card: 'summary_large_image',
    title: 'AI Memory for Edge Devices - Robots, Drones & IoT',
    description: '100% offline AI memory. 4MB binary, <100ms retrieval. Built for defence, drones, and industrial robots.',
    images: ['/og-image.png'],
  },
  alternates: {
    canonical: 'https://shodh-rag.com/memory',
  },
}

export default function MemoryLayout({
  children,
}: {
  children: React.ReactNode
}) {
  return children
}
