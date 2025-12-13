import { Metadata } from 'next'

export const metadata: Metadata = {
  title: 'Pricing - Shodh Memory & Shodh RAG',
  description: 'Shodh Memory: Free open source forever. Shodh RAG: Free pilot, enterprise pricing. On-premise deployment, no cloud dependency. Start free, scale when ready.',
  keywords: [
    'RAG pricing', 'AI memory pricing', 'enterprise RAG cost',
    'open source AI memory', 'on-premise RAG pricing', 'free AI tools'
  ],
  openGraph: {
    title: 'Shodh Pricing - Free Open Source + Enterprise Options',
    description: 'Shodh Memory: Free forever. Shodh RAG: Free pilot. Enterprise support available.',
    url: 'https://shodh-rag.com/pricing',
  },
}

export default function PricingLayout({
  children,
}: {
  children: React.ReactNode
}) {
  return children
}
