import { Metadata } from 'next'

export const metadata: Metadata = {
  title: 'RAG for Robotics - Local AI for Robots & Drones | Shodh',
  description: 'Give robots a local AI brain with Shodh RAG. 50ms response time, 100% offline, ROS2/Zenoh integration. Perfect for manufacturing, warehousing, agriculture, and defence robotics.',
  keywords: [
    'RAG for robotics', 'robot AI', 'drone AI', 'ROS2 RAG',
    'Zenoh robotics', 'offline robotics AI', 'local robot intelligence',
    'manufacturing robots', 'warehouse automation', 'agricultural robots',
    'autonomous robots', 'robot knowledge base', 'embodied AI',
    'robot decision making', 'fleet coordination', 'multi-robot systems',
    'industrial automation', 'robot natural language', 'robot control',
    'edge robotics', 'local LLM robotics', 'robot memory'
  ],
  openGraph: {
    title: 'RAG for Robotics - Local AI Brain for Robots & Drones',
    description: '50ms response time, 100% offline operation. Shodh RAG gives robots context-aware intelligence without cloud dependency. ROS2 + Zenoh integration.',
    type: 'website',
    url: 'https://shodhrag.com/robotics',
    images: [
      {
        url: '/og-image.png',
        width: 1200,
        height: 630,
        alt: 'Shodh RAG for Robotics - Local AI for Robots and Drones',
      },
    ],
  },
  twitter: {
    card: 'summary_large_image',
    title: 'RAG for Robotics - Local AI Brain for Robots',
    description: '50ms response, 100% offline. ROS2 + Zenoh integration. Built for manufacturing, warehouse, and agricultural robots.',
    images: ['/og-image.png'],
  },
  alternates: {
    canonical: 'https://shodhrag.com/robotics',
  },
}

export default function RoboticsLayout({
  children,
}: {
  children: React.ReactNode
}) {
  return children
}
