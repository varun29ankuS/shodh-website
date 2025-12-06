import { Metadata } from 'next';

export const metadata: Metadata = {
  title: 'Blog | Shodh',
  description: 'Technical insights on AI memory systems, robotics, edge computing, and building production-grade RAG systems. Learn from real-world implementations.',
  keywords: [
    'AI blog', 'robotics engineering', 'edge AI', 'memory systems',
    'ROS2 tutorials', 'vector database', 'RAG systems', 'Rust AI'
  ],
  openGraph: {
    title: 'Shodh Engineering Blog',
    description: 'Technical insights on AI memory systems, robotics, and edge computing',
    type: 'website',
  },
};

export default function BlogLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return children;
}
