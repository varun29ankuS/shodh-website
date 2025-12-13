import { BlogPost } from './blog';

export const blogPosts: BlogPost[] = [
  {
    slug: 'shodh-memory-python-sdk-complete-tutorial',
    title: 'Shodh Memory Python SDK: Complete Tutorial with Examples',
    description: 'Learn how to use the Shodh Memory Python SDK from installation to advanced features. Store memories, semantic recall, context summaries, and integration patterns for AI applications.',
    date: '2025-12-13',
    author: {
      name: 'Shodh Team',
      role: 'Developer Experience',
    },
    category: 'Tutorial',
    tags: ['python', 'SDK', 'tutorial', 'shodh-memory', 'AI-memory'],
    readingTime: '8 min read',
    featured: true,
  },
  {
    slug: 'how-shodh-memory-works-hebbian-learning',
    title: 'How Shodh Memory Works: Hebbian Learning & Cognitive Architecture',
    description: 'Deep dive into the cognitive science behind Shodh Memory. Hebbian plasticity, spreading activation, memory consolidation, and the three-tier architecture that makes AI memory human-like.',
    date: '2025-12-12',
    author: {
      name: 'Shodh Team',
      role: 'AI Research',
    },
    category: 'AI',
    tags: ['Hebbian-learning', 'cognitive-science', 'architecture', 'memory-systems', 'AI'],
    readingTime: '10 min read',
    featured: true,
  },
  {
    slug: 'ai-memory-use-cases-developer-workflows',
    title: '5 Ways AI Memory Transforms Your Development Workflow',
    description: 'Practical use cases for persistent AI memory: project context retention, preference learning, error tracking, knowledge accumulation, and cross-session continuity. Real examples with Claude Code.',
    date: '2025-12-11',
    author: {
      name: 'Shodh Team',
      role: 'Developer Advocacy',
    },
    category: 'AI',
    tags: ['use-cases', 'developer-tools', 'productivity', 'Claude-Code', 'workflows'],
    readingTime: '6 min read',
    featured: false,
  },
  {
    slug: 'claude-code-mcp-server-setup-guide',
    title: 'How to Set Up MCP Servers in Claude Code: Complete Guide',
    description: 'Learn how to supercharge Claude Code with MCP (Model Context Protocol) servers. From basic setup to production deployment, with practical examples using Shodh Memory for persistent AI memory.',
    date: '2025-12-09',
    author: {
      name: 'Shodh Team',
      role: 'Developer Experience',
    },
    category: 'Tutorial',
    tags: ['claude-code', 'MCP', 'tutorial', 'AI-agents', 'developer-tools'],
    readingTime: '10 min read',
    featured: true,
  },
  {
    slug: 'agentic-ai-long-term-memory-2025',
    title: 'Why Long-Term Memory is the Missing Piece in Agentic AI Systems',
    description: 'Gartner named agentic AI the #1 technology trend for 2025. But without persistent memory, AI agents forget everything between sessions. Here\'s how to fix that with local-first memory systems.',
    date: '2025-12-06',
    author: {
      name: 'Shodh Team',
      role: 'AI Engineering',
    },
    category: 'AI',
    tags: ['agentic-ai', 'long-term-memory', 'LLM', 'context-engineering', 'multi-agent'],
    readingTime: '8 min read',
    featured: false,
  },
  {
    slug: 'embodied-ai-edge-memory-robotics',
    title: 'Embodied AI Needs Edge Memory: Why Cloud-Dependent Robots Fail',
    description: 'The embodied AI market is projected to hit $23B by 2030. But humanoids and autonomous systems can\'t rely on cloud latency. Learn how edge-native memory enables real-time decision making.',
    date: '2025-12-05',
    author: {
      name: 'Shodh Team',
      role: 'Robotics',
    },
    category: 'Robotics',
    tags: ['embodied-ai', 'edge-computing', 'humanoid-robots', 'world-models', 'VLA'],
    readingTime: '7 min read',
    featured: true,
  },
  {
    slug: 'autonomous-robot-memory-ros2-guide',
    title: 'Building Autonomous Robots with Persistent Memory: A ROS2 Implementation Guide',
    description: 'From warehouse AGVs to delivery drones, autonomous systems need to remember failures, learn from experience, and adapt. A practical guide to implementing Hebbian learning in ROS2 robots.',
    date: '2025-12-04',
    author: {
      name: 'Shodh Team',
      role: 'Engineering',
    },
    category: 'Tutorial',
    tags: ['ROS2', 'autonomous-systems', 'Hebbian-learning', 'Python', 'robotics'],
    readingTime: '12 min read',
  },
];

export function getAllPosts(): BlogPost[] {
  return blogPosts.sort((a, b) => new Date(b.date).getTime() - new Date(a.date).getTime());
}

export function getPostBySlug(slug: string): BlogPost | undefined {
  return blogPosts.find((post) => post.slug === slug);
}

export function getFeaturedPosts(): BlogPost[] {
  return blogPosts.filter((post) => post.featured);
}

export function getPostsByCategory(category: BlogPost['category']): BlogPost[] {
  return blogPosts.filter((post) => post.category === category);
}

export function getPostsByTag(tag: string): BlogPost[] {
  return blogPosts.filter((post) => post.tags.includes(tag));
}
