'use client'

import { motion } from 'framer-motion'
import { Play, Pause, Volume2, VolumeX } from 'lucide-react'
import { useState, useRef } from 'react'

export default function VideoHero() {
  const [isPlaying, setIsPlaying] = useState(false)
  const [isMuted, setIsMuted] = useState(true)
  const videoRef = useRef<HTMLVideoElement>(null)

  const togglePlay = () => {
    if (videoRef.current) {
      if (isPlaying) {
        videoRef.current.pause()
      } else {
        videoRef.current.play()
      }
      setIsPlaying(!isPlaying)
    }
  }

  const toggleMute = () => {
    if (videoRef.current) {
      videoRef.current.muted = !isMuted
      setIsMuted(!isMuted)
    }
  }

  return (
    <section className="py-24 bg-gradient-to-b from-white to-slate-50 dark:from-slate-950 dark:to-slate-900">
      <div className="container mx-auto px-4">
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.5 }}
          viewport={{ once: true }}
          className="text-center mb-12"
        >
          <h2 className="text-4xl md:text-5xl font-bold mb-4 text-slate-900 dark:text-white">
            See SHODH in Action
          </h2>
          <p className="text-xl text-slate-600 dark:text-slate-400 max-w-2xl mx-auto">
            Watch how teams use SHODH to power their document intelligence
          </p>
        </motion.div>

        <motion.div
          initial={{ opacity: 0, scale: 0.95 }}
          whileInView={{ opacity: 1, scale: 1 }}
          transition={{ duration: 0.5, delay: 0.2 }}
          viewport={{ once: true }}
          className="relative max-w-5xl mx-auto rounded-2xl overflow-hidden shadow-2xl border-4 border-primary/20"
        >
          {/* Video Player */}
          <div className="relative aspect-video bg-slate-900">
            {/* Placeholder - Replace with actual video */}
            <video
              ref={videoRef}
              className="w-full h-full object-cover"
              poster="/video-poster.jpg"
              loop
              playsInline
              muted={isMuted}
            >
              <source src="/demo-video.mp4" type="video/mp4" />
              <source src="/demo-video.webm" type="video/webm" />
              Your browser does not support the video tag.
            </video>

            {/* Overlay with play button */}
            {!isPlaying && (
              <motion.div
                initial={{ opacity: 0 }}
                animate={{ opacity: 1 }}
                className="absolute inset-0 flex items-center justify-center bg-black/30 cursor-pointer"
                onClick={togglePlay}
              >
                <motion.button
                  whileHover={{ scale: 1.1 }}
                  whileTap={{ scale: 0.95 }}
                  className="w-20 h-20 bg-primary rounded-full flex items-center justify-center shadow-2xl hover:bg-primary/90 transition-colors"
                >
                  <Play className="w-10 h-10 text-white ml-1" />
                </motion.button>
              </motion.div>
            )}

            {/* Video Controls */}
            {isPlaying && (
              <motion.div
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                className="absolute bottom-4 left-4 right-4 flex items-center gap-4"
              >
                <button
                  onClick={togglePlay}
                  className="p-2 bg-black/50 hover:bg-black/70 rounded-lg backdrop-blur-sm transition-colors"
                >
                  <Pause className="w-5 h-5 text-white" />
                </button>
                <button
                  onClick={toggleMute}
                  className="p-2 bg-black/50 hover:bg-black/70 rounded-lg backdrop-blur-sm transition-colors"
                >
                  {isMuted ? (
                    <VolumeX className="w-5 h-5 text-white" />
                  ) : (
                    <Volume2 className="w-5 h-5 text-white" />
                  )}
                </button>
              </motion.div>
            )}
          </div>

          {/* Decorative border glow */}
          <div className="absolute -inset-1 bg-gradient-to-r from-primary via-secondary to-destructive rounded-2xl blur-xl opacity-20 -z-10"></div>
        </motion.div>

        {/* Video Stats/Info */}
        <motion.div
          initial={{ opacity: 0, y: 20 }}
          whileInView={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.5, delay: 0.4 }}
          viewport={{ once: true }}
          className="mt-12 grid grid-cols-1 md:grid-cols-3 gap-6 max-w-4xl mx-auto"
        >
          {[
            { label: 'Setup Time', value: '< 5 minutes' },
            { label: 'Query Speed', value: '< 10ms' },
            { label: 'Citation Accuracy', value: '97%' },
          ].map((stat) => (
            <div key={stat.label} className="text-center p-6 rounded-xl bg-white dark:bg-slate-900 border border-slate-200 dark:border-slate-800">
              <div className="text-3xl font-bold text-primary mb-2">{stat.value}</div>
              <div className="text-sm text-slate-600 dark:text-slate-400">{stat.label}</div>
            </div>
          ))}
        </motion.div>
      </div>
    </section>
  )
}
