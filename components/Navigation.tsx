'use client'

import { useState } from 'react'
import Link from 'next/link'
import Image from 'next/image'
import { usePathname } from 'next/navigation'
import { Menu, X, Moon, Sun } from 'lucide-react'
import { useTheme } from '@/contexts/ThemeContext'

export default function Navigation() {
  const [isOpen, setIsOpen] = useState(false)
  const { theme, toggleTheme } = useTheme()
  const pathname = usePathname()

  const navigation = [
    { name: 'Home', href: '/' },
    { name: 'Shodh RAG', href: '/features' },
    { name: 'Shodh Memory', href: '/memory' },
    { name: 'Pricing', href: '/pricing' },
    { name: 'Robotics', href: '/robotics' },
    { name: 'Blog', href: '/blog' },
    { name: 'Docs', href: '/docs' },
  ]

  return (
    <nav className="fixed top-0 left-0 right-0 z-50 bg-white/80 dark:bg-slate-950/80 backdrop-blur-md border-b border-slate-200 dark:border-slate-800">
      <div className="container mx-auto px-4">
        <div className="flex items-center justify-between h-16">
          {/* Logo */}
          <Link href="/" className="flex items-center gap-4 hover:opacity-90 transition-opacity">
            <div className="relative w-14 h-14">
              <Image
                src="/logo.png"
                alt="Shodh Logo"
                fill
                className="object-contain"
                priority
              />
            </div>
            <div className="flex flex-col">
              <span className="text-2xl font-bold text-slate-900 dark:text-white">SHODH</span>
              <span className="text-sm text-primary font-semibold">शोध</span>
            </div>
          </Link>

          {/* Desktop Navigation */}
          <div className="hidden md:flex items-center gap-8">
            {navigation.map((item) => {
              const isActive = pathname === item.href
              return (
                <Link
                  key={item.name}
                  href={item.href}
                  className={`text-sm font-medium transition-colors relative ${
                    isActive
                      ? 'text-primary dark:text-primary font-bold'
                      : 'text-slate-600 dark:text-slate-300 hover:text-primary dark:hover:text-primary'
                  }`}
                >
                  {item.name}
                  {isActive && (
                    <span className="absolute -bottom-1 left-0 right-0 h-0.5 bg-primary rounded-full" />
                  )}
                </Link>
              )
            })}
            <button
              onClick={toggleTheme}
              className="p-2 rounded-lg hover:bg-slate-100 dark:hover:bg-slate-800 transition-colors"
              aria-label="Toggle theme"
            >
              {theme === 'dark' ? (
                <Sun className="w-5 h-5 text-slate-600 dark:text-slate-300" />
              ) : (
                <Moon className="w-5 h-5 text-slate-600" />
              )}
            </button>
            <Link
              href="/demo"
              className="px-6 py-2 bg-primary hover:bg-primary/90 text-white rounded-lg font-semibold transition-all transform hover:scale-105"
            >
              Try Demo
            </Link>
          </div>

          {/* Mobile menu button */}
          <div className="flex items-center gap-2 md:hidden">
            <button
              onClick={toggleTheme}
              className="p-2 rounded-lg hover:bg-slate-100 dark:hover:bg-slate-800 transition-colors"
              aria-label="Toggle theme"
            >
              {theme === 'dark' ? (
                <Sun className="w-5 h-5 text-slate-600 dark:text-slate-300" />
              ) : (
                <Moon className="w-5 h-5 text-slate-600" />
              )}
            </button>
            <button
              onClick={() => setIsOpen(!isOpen)}
              className="p-2 rounded-lg hover:bg-slate-100 dark:hover:bg-slate-800 transition-colors"
              aria-label="Toggle menu"
            >
              {isOpen ? (
                <X className="w-6 h-6 text-slate-600 dark:text-slate-300" />
              ) : (
                <Menu className="w-6 h-6 text-slate-600 dark:text-slate-300" />
              )}
            </button>
          </div>
        </div>

        {/* Mobile menu */}
        {isOpen && (
          <div className="md:hidden py-4 border-t border-slate-200 dark:border-slate-800">
            <div className="flex flex-col gap-4">
              {navigation.map((item) => {
                const isActive = pathname === item.href
                return (
                  <Link
                    key={item.name}
                    href={item.href}
                    className={`text-sm font-medium transition-colors relative pl-4 ${
                      isActive
                        ? 'text-primary dark:text-primary font-bold border-l-2 border-primary'
                        : 'text-slate-600 dark:text-slate-300 hover:text-primary dark:hover:text-primary'
                    }`}
                    onClick={() => setIsOpen(false)}
                  >
                    {item.name}
                  </Link>
                )
              })}
              <Link
                href="/demo"
                className="px-6 py-2 bg-primary hover:bg-primary/90 text-white rounded-lg font-semibold transition-colors text-center"
                onClick={() => setIsOpen(false)}
              >
                Try Demo
              </Link>
            </div>
          </div>
        )}
      </div>
    </nav>
  )
}
