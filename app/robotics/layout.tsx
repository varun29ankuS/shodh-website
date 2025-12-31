import { Metadata } from 'next'

export const metadata: Metadata = {
  title: 'Edge AI Memory for Robots, Drones & IoT | Shodh Memory',
  description: 'Give robots persistent memory that learns from experience. Hebbian learning, 100% offline, single ~18MB binary. Perfect for industrial robots, warehouse AGVs, drones, and agricultural automation.',
  keywords: [
    'robot memory', 'drone memory', 'IoT memory', 'edge AI',
    'offline robot AI', 'Hebbian learning robotics', 'experience-based learning',
    'manufacturing robots', 'warehouse AGV', 'agricultural robots',
    'autonomous systems', 'robot knowledge graph', 'embodied AI',
    'robot decision making', 'industrial automation', 'ROS2 memory',
    'air-gapped AI', 'defence robotics', 'Jetson AI', 'Raspberry Pi AI',
    'persistent robot memory', 'robot experience learning'
  ],
  openGraph: {
    title: 'Edge AI Memory for Robots, Drones & IoT',
    description: 'Robots that learn from experience. Hebbian learning on-device, 100% offline, single ~18MB binary. Built for industrial, warehouse, and agricultural automation.',
    type: 'website',
    url: 'https://shodh-rag.com/robotics',
    images: [
      {
        url: '/og-image.png',
        width: 1200,
        height: 630,
        alt: 'Shodh Memory - Edge AI Memory for Robots and Drones',
      },
    ],
  },
  twitter: {
    card: 'summary_large_image',
    title: 'Edge AI Memory for Robots & Drones',
    description: 'Robots that learn from experience. Hebbian learning, 100% offline. Built for industrial, warehouse, and agricultural automation.',
    images: ['/og-image.png'],
  },
  alternates: {
    canonical: 'https://shodh-rag.com/robotics',
  },
}

export default function RoboticsLayout({
  children,
}: {
  children: React.ReactNode
}) {
  return children
}
