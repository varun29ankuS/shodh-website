import { Metadata } from 'next'

export const metadata: Metadata = {
  title: 'Try Shodh - Interactive Demo & Free Trial',
  description: 'Try Shodh Memory in Google Colab - no installation required. Or book a personalized demo to see Shodh RAG on your documents. Free pilot available.',
  keywords: [
    'try shodh', 'AI memory demo', 'RAG demo', 'Google Colab AI',
    'free AI trial', 'document search demo', 'enterprise AI demo'
  ],
  openGraph: {
    title: 'Try Shodh - Free Interactive Demo',
    description: 'Try in Google Colab instantly or book a personalized demo. See Shodh on your documents.',
    url: 'https://shodh-rag.com/demo',
  },
}

export default function DemoLayout({
  children,
}: {
  children: React.ReactNode
}) {
  return children
}
