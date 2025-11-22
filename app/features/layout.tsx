import { Metadata } from 'next'

export const metadata: Metadata = {
  title: 'Features - Production-Grade RAG on Your Laptop | Shodh',
  description: '97% citation accuracy, <10ms query latency, on-premise deployment. Enterprise RAG features: hybrid search, multi-tenant, SIMD-optimized vectors, RocksDB/LMDB storage. No cloud dependency.',
  keywords: [
    'RAG features', 'production RAG', 'on-premise RAG', 'local RAG',
    'hybrid search', 'vector database', 'citation accuracy',
    'enterprise RAG', 'SIMD optimization', 'multi-tenant RAG',
    'offline RAG', 'private RAG', 'code-aware RAG',
    'RocksDB vector store', 'LMDB vector database',
    'fast vector search', 'local vector database', 'RAG monitoring'
  ],
  openGraph: {
    title: 'Shodh RAG Features - Production-Grade Local RAG',
    description: '97% citation accuracy, <10ms queries, on-premise deployment. Enterprise features for teams who need RAG without cloud dependency.',
    type: 'website',
    url: 'https://shodh-rag.com/features',
    images: [
      {
        url: '/og-image.png',
        width: 1200,
        height: 630,
        alt: 'Shodh RAG Features - Production-Grade Local RAG System',
      },
    ],
  },
  twitter: {
    card: 'summary_large_image',
    title: 'Shodh RAG Features - Production-Grade Local RAG',
    description: '97% citation accuracy, <10ms queries. Enterprise RAG without cloud dependency.',
    images: ['/og-image.png'],
  },
  alternates: {
    canonical: 'https://shodh-rag.com/features',
  },
}

export default function FeaturesLayout({
  children,
}: {
  children: React.ReactNode
}) {
  return children
}
