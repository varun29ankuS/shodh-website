import './globals.css'
import { Inter } from 'next/font/google'
import { ThemeProvider } from '@/contexts/ThemeContext'
import Navigation from '@/components/Navigation'
import Footer from '@/components/Footer'
import { Analytics } from '@vercel/analytics/next'

const inter = Inter({ subsets: ['latin'] })

export const metadata = {
  metadataBase: new URL('https://shodhrag.com'),
  title: {
    default: 'Shodh RAG - Production RAG on Your Existing Laptop | Local Vector Database',
    template: '%s | Shodh RAG',
  },
  icons: {
    icon: '/logo.png',
    shortcut: '/logo.png',
    apple: '/logo.png',
  },
  description: 'Run production RAG on your existing laptop. Index 1000 documents locally, search in 50-80ms, choose LLM flexibility (free local or paid APIs). 90% cost savings vs cloud. No GPU required. Built for India.',
  keywords: [
    'RAG', 'local RAG', 'on-premise RAG', 'laptop RAG', 'offline RAG',
    'vector database', 'local vector db', 'Rust vector database',
    'LLM flexibility', 'local LLM', 'llama.cpp', 'ONNX inference',
    'enterprise search', 'document search', 'knowledge management',
    'Hindi RAG', 'multilingual RAG', 'Indian languages',
    'robotics AI', 'Zenoh', 'ROS2', 'embedded AI',
    'cost-effective AI', 'no cloud dependency', 'data sovereignty',
    'Vamana', 'BM25', 'hybrid search', 'CPU-optimized AI'
  ],
  authors: [{ name: 'Shodh Team', url: 'https://shodhrag.com' }],
  creator: 'Shodh',
  publisher: 'Shodh',
  formatDetection: {
    email: false,
    address: false,
    telephone: false,
  },
  openGraph: {
    type: 'website',
    locale: 'en_IN',
    url: 'https://shodhrag.com',
    title: 'Shodh RAG - Production RAG on Your Laptop | 90% Cost Savings',
    description: 'Index 1000 docs on 16GB laptop. 50-80ms queries. Free local LLMs or paid APIs. No cloud needed. Built for India with Hindi support.',
    siteName: 'Shodh',
    images: [
      {
        url: '/og-image.png',
        width: 1200,
        height: 630,
        alt: 'Shodh RAG - Run production RAG on your existing laptop',
      },
    ],
  },
  twitter: {
    card: 'summary_large_image',
    title: 'Shodh RAG - Production RAG on Your Laptop',
    description: '1000 docs on 16GB laptop. 50-80ms queries. LLM flexibility. 90% cost savings. Built for India.',
    images: ['/og-image.png'],
    creator: '@shodhAI',
  },
  robots: {
    index: true,
    follow: true,
    googleBot: {
      index: true,
      follow: true,
      'max-video-preview': -1,
      'max-image-preview': 'large',
      'max-snippet': -1,
    },
  },
  verification: {
    // Add your verification codes here when you have them
    // google: 'your-google-verification-code',
    // yandex: 'your-yandex-verification-code',
  },
}

export default function RootLayout({
  children,
}: {
  children: React.ReactNode
}) {
  return (
    <html lang="en" suppressHydrationWarning>
      <body className={inter.className}>
        <ThemeProvider>
          <Navigation />
          <main className="pt-16">
            {children}
          </main>
          <Footer />
          <Analytics />
        </ThemeProvider>
      </body>
    </html>
  )
}
