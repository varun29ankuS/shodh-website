'use client';

import { motion } from 'framer-motion';
import { Rss, Search } from 'lucide-react';
import BlogCard from '@/components/BlogCard';
import { getAllPosts, getFeaturedPosts } from '@/lib/posts';

export default function BlogPage() {
  const allPosts = getAllPosts();
  const featuredPosts = getFeaturedPosts();

  return (
    <div className="min-h-screen bg-gradient-to-b from-slate-50 to-white dark:from-slate-900 dark:to-slate-800">
      {/* Hero Section */}
      <section className="pt-20 pb-16 px-4">
        <div className="max-w-6xl mx-auto text-center">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.6 }}
          >
            <span className="inline-block px-4 py-1.5 bg-orange-100 dark:bg-orange-900/30 text-orange-700 dark:text-orange-300 rounded-full text-sm font-medium mb-6">
              Blog
            </span>
            <h1 className="text-4xl md:text-5xl font-bold text-slate-900 dark:text-white mb-6">
              AI That Remembers,{' '}
              <span className="bg-gradient-to-r from-orange-600 to-red-600 bg-clip-text text-transparent">
                Documents That Talk
              </span>
            </h1>
            <p className="text-xl text-slate-600 dark:text-slate-300 max-w-3xl mx-auto mb-8">
              Practical guides, tutorials, and insights on AI memory systems, document intelligence,
              and building AI that actually works for your business.
            </p>

            <div className="flex items-center justify-center gap-4">
              <a
                href="/blog/rss.xml"
                className="flex items-center gap-2 px-4 py-2 text-sm text-slate-600 dark:text-slate-300 hover:text-orange-600 dark:hover:text-orange-400 transition-colors"
              >
                <Rss className="w-4 h-4" />
                RSS Feed
              </a>
            </div>
          </motion.div>
        </div>
      </section>

      {/* Featured Posts */}
      {featuredPosts.length > 0 && (
        <section className="pb-16 px-4">
          <div className="max-w-6xl mx-auto">
            <h2 className="text-2xl font-bold text-slate-900 dark:text-white mb-8 flex items-center gap-2">
              <span className="w-8 h-1 bg-orange-500 rounded" />
              Featured
            </h2>
            <div className="grid md:grid-cols-2 gap-8">
              {featuredPosts.map((post, index) => (
                <BlogCard key={post.slug} post={post} index={index} />
              ))}
            </div>
          </div>
        </section>
      )}

      {/* All Posts */}
      <section className="pb-20 px-4">
        <div className="max-w-6xl mx-auto">
          <h2 className="text-2xl font-bold text-slate-900 dark:text-white mb-8 flex items-center gap-2">
            <span className="w-8 h-1 bg-slate-400 dark:bg-slate-600 rounded" />
            All Posts
          </h2>
          <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-8">
            {allPosts.map((post, index) => (
              <BlogCard key={post.slug} post={post} index={index} />
            ))}
          </div>
        </div>
      </section>

      {/* Newsletter CTA */}
      <section className="pb-20 px-4">
        <div className="max-w-4xl mx-auto">
          <div className="bg-gradient-to-br from-orange-500 to-red-600 rounded-2xl p-8 md:p-12 text-center text-white">
            <h2 className="text-2xl md:text-3xl font-bold mb-4">
              Stay Updated
            </h2>
            <p className="text-orange-100 mb-6 max-w-2xl mx-auto">
              Get notified when we publish new guides, tutorials, and product updates.
              Follow us on GitHub for the latest releases.
            </p>
            <a
              href="https://github.com/varun29ankuS/shodh-memory"
              target="_blank"
              rel="noopener noreferrer"
              className="inline-flex items-center gap-2 px-6 py-3 bg-white text-orange-600 font-medium rounded-lg hover:bg-orange-50 transition-colors"
            >
              Star on GitHub
            </a>
          </div>
        </div>
      </section>
    </div>
  );
}
