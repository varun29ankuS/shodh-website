export default function StructuredData() {
  const organizationSchema = {
    '@context': 'https://schema.org',
    '@type': 'Organization',
    name: 'Shodh',
    alternateName: 'Shodh RAG',
    url: 'https://shodh-rag.com',
    logo: {
      '@type': 'ImageObject',
      url: 'https://shodh-rag.com/logo.png',
      width: 1024,
      height: 1024,
      caption: 'Shodh Logo',
    },
    image: 'https://shodh-rag.com/og-image.png',
    description: 'Production-grade RAG system that runs on your laptop. AI memory engine for edge devices, robots, drones, and IoT.',
    sameAs: [
      'https://twitter.com/shodhAI',
    ],
  }

  const softwareSchema = {
    '@context': 'https://schema.org',
    '@type': 'SoftwareApplication',
    name: 'Shodh RAG',
    applicationCategory: 'DeveloperApplication',
    operatingSystem: 'Windows, macOS, Linux',
    offers: {
      '@type': 'Offer',
      price: '0',
      priceCurrency: 'USD',
    },
    description: 'Production RAG on your laptop. Index 1000 documents locally, search in 50-80ms, choose LLM flexibility (free local or paid APIs). 90% cost savings vs cloud. No GPU required.',
    featureList: [
      '97% citation accuracy',
      'Sub-100ms query latency',
      '100% offline operation',
      'On-premise deployment',
      'ROS2 and Zenoh integration',
      'Edge AI memory for robots and drones',
      '3-tier memory hierarchy',
      'Knowledge graph support',
    ],
    softwareRequirements: '16GB RAM, 4 CPU cores recommended',
    screenshot: 'https://shodh-rag.com/og-image.png',
  }

  const productSchema = {
    '@context': 'https://schema.org',
    '@type': 'Product',
    name: 'Shodh Memory',
    description: 'Edge-native AI memory system for robots, drones, and IoT devices. 100% offline, ~18MB binary, <100ms retrieval.',
    image: 'https://shodh-rag.com/og-image.png',
    brand: {
      '@type': 'Brand',
      name: 'Shodh',
    },
    offers: {
      '@type': 'Offer',
      price: '0',
      priceCurrency: 'USD',
      availability: 'https://schema.org/InStock',
    },
    category: 'Edge AI Software',
  }

  return (
    <>
      <script
        type="application/ld+json"
        dangerouslySetInnerHTML={{ __html: JSON.stringify(organizationSchema) }}
      />
      <script
        type="application/ld+json"
        dangerouslySetInnerHTML={{ __html: JSON.stringify(softwareSchema) }}
      />
      <script
        type="application/ld+json"
        dangerouslySetInnerHTML={{ __html: JSON.stringify(productSchema) }}
      />
    </>
  )
}
