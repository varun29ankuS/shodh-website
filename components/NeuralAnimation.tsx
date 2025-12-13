'use client'

import { useEffect, useRef } from 'react'
import { motion } from 'framer-motion'

interface Neuron {
  x: number
  y: number
  vx: number
  vy: number
  activation: number
  targetActivation: number
  connections: number[]
}

export default function NeuralAnimation() {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const neuronsRef = useRef<Neuron[]>([])
  const animationRef = useRef<number>(0)
  const mouseRef = useRef({ x: -1000, y: -1000 })

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return

    const ctx = canvas.getContext('2d')
    if (!ctx) return

    const resizeCanvas = () => {
      canvas.width = canvas.offsetWidth * window.devicePixelRatio
      canvas.height = canvas.offsetHeight * window.devicePixelRatio
      ctx.scale(window.devicePixelRatio, window.devicePixelRatio)
    }

    resizeCanvas()
    window.addEventListener('resize', resizeCanvas)

    // Initialize neurons
    const neuronCount = 40
    const neurons: Neuron[] = []

    for (let i = 0; i < neuronCount; i++) {
      neurons.push({
        x: Math.random() * canvas.offsetWidth,
        y: Math.random() * canvas.offsetHeight,
        vx: (Math.random() - 0.5) * 0.3,
        vy: (Math.random() - 0.5) * 0.3,
        activation: Math.random() * 0.3,
        targetActivation: 0,
        connections: []
      })
    }

    // Create connections (Hebbian-style: nearby neurons connect)
    for (let i = 0; i < neurons.length; i++) {
      for (let j = i + 1; j < neurons.length; j++) {
        const dx = neurons[i].x - neurons[j].x
        const dy = neurons[i].y - neurons[j].y
        const dist = Math.sqrt(dx * dx + dy * dy)
        if (dist < 150) {
          neurons[i].connections.push(j)
          neurons[j].connections.push(i)
        }
      }
    }

    neuronsRef.current = neurons

    // Mouse tracking
    const handleMouseMove = (e: MouseEvent) => {
      const rect = canvas.getBoundingClientRect()
      mouseRef.current = {
        x: e.clientX - rect.left,
        y: e.clientY - rect.top
      }
    }

    const handleMouseLeave = () => {
      mouseRef.current = { x: -1000, y: -1000 }
    }

    canvas.addEventListener('mousemove', handleMouseMove)
    canvas.addEventListener('mouseleave', handleMouseLeave)

    // Animation loop
    const animate = () => {
      ctx.clearRect(0, 0, canvas.offsetWidth, canvas.offsetHeight)

      const mouse = mouseRef.current
      const neurons = neuronsRef.current

      // Update neurons
      for (let i = 0; i < neurons.length; i++) {
        const neuron = neurons[i]

        // Move neurons slowly
        neuron.x += neuron.vx
        neuron.y += neuron.vy

        // Bounce off edges
        if (neuron.x < 0 || neuron.x > canvas.offsetWidth) neuron.vx *= -1
        if (neuron.y < 0 || neuron.y > canvas.offsetHeight) neuron.vy *= -1

        // Keep in bounds
        neuron.x = Math.max(0, Math.min(canvas.offsetWidth, neuron.x))
        neuron.y = Math.max(0, Math.min(canvas.offsetHeight, neuron.y))

        // Activation based on mouse proximity (spreading activation)
        const dx = mouse.x - neuron.x
        const dy = mouse.y - neuron.y
        const distToMouse = Math.sqrt(dx * dx + dy * dy)

        if (distToMouse < 120) {
          neuron.targetActivation = 1 - (distToMouse / 120)
        } else {
          neuron.targetActivation = 0.1 + Math.sin(Date.now() * 0.001 + i) * 0.1
        }

        // Smooth activation transition
        neuron.activation += (neuron.targetActivation - neuron.activation) * 0.08

        // Hebbian: spread activation to connected neurons
        if (neuron.activation > 0.5) {
          for (const connIdx of neuron.connections) {
            neurons[connIdx].targetActivation = Math.max(
              neurons[connIdx].targetActivation,
              neuron.activation * 0.6
            )
          }
        }
      }

      // Draw connections first (behind neurons)
      for (let i = 0; i < neurons.length; i++) {
        const neuron = neurons[i]
        for (const j of neuron.connections) {
          if (j > i) {
            const other = neurons[j]
            const avgActivation = (neuron.activation + other.activation) / 2

            // Only draw if there's some activation
            if (avgActivation > 0.15) {
              ctx.beginPath()
              ctx.moveTo(neuron.x, neuron.y)
              ctx.lineTo(other.x, other.y)

              // Connection strength based on activation (Hebbian visualization)
              const alpha = avgActivation * 0.4
              const lineWidth = avgActivation * 2

              ctx.strokeStyle = `rgba(59, 130, 246, ${alpha})`
              ctx.lineWidth = lineWidth
              ctx.stroke()

              // Draw "signal" traveling along strong connections
              if (avgActivation > 0.5) {
                const t = (Date.now() * 0.002 + i * 0.1) % 1
                const signalX = neuron.x + (other.x - neuron.x) * t
                const signalY = neuron.y + (other.y - neuron.y) * t

                ctx.beginPath()
                ctx.arc(signalX, signalY, 2, 0, Math.PI * 2)
                ctx.fillStyle = `rgba(59, 130, 246, ${avgActivation})`
                ctx.fill()
              }
            }
          }
        }
      }

      // Draw neurons
      for (const neuron of neurons) {
        // Outer glow
        const gradient = ctx.createRadialGradient(
          neuron.x, neuron.y, 0,
          neuron.x, neuron.y, 15 + neuron.activation * 10
        )
        gradient.addColorStop(0, `rgba(59, 130, 246, ${neuron.activation * 0.5})`)
        gradient.addColorStop(1, 'rgba(59, 130, 246, 0)')

        ctx.beginPath()
        ctx.arc(neuron.x, neuron.y, 15 + neuron.activation * 10, 0, Math.PI * 2)
        ctx.fillStyle = gradient
        ctx.fill()

        // Core
        ctx.beginPath()
        ctx.arc(neuron.x, neuron.y, 3 + neuron.activation * 3, 0, Math.PI * 2)
        ctx.fillStyle = `rgba(59, 130, 246, ${0.3 + neuron.activation * 0.7})`
        ctx.fill()
      }

      animationRef.current = requestAnimationFrame(animate)
    }

    animate()

    return () => {
      window.removeEventListener('resize', resizeCanvas)
      canvas.removeEventListener('mousemove', handleMouseMove)
      canvas.removeEventListener('mouseleave', handleMouseLeave)
      cancelAnimationFrame(animationRef.current)
    }
  }, [])

  return (
    <motion.canvas
      ref={canvasRef}
      initial={{ opacity: 0 }}
      animate={{ opacity: 1 }}
      transition={{ duration: 1, delay: 0.5 }}
      className="absolute inset-0 w-full h-full pointer-events-auto"
      style={{ zIndex: 1 }}
    />
  )
}
